// 단계 4 — Unity 프로젝트 설정과 씬 구성 (에디터 전용, batchmode 에서 -executeMethod 로 실행).
//
//   "$UNITY_BIN" -batchmode -nographics -projectPath UnityProject -executeMethod SimSetup.ConfigureProject -quit
//   "$UNITY_BIN" -batchmode -nographics -projectPath UnityProject -executeMethod SimSetup.BuildScene -quit
//
// ConfigureProject 와 BuildScene 은 반드시 별도 실행이다. ConfigureProject 가 넣는 스크립팅 정의 심볼 ROS2 는
// 메서드가 반환된 뒤 재컴파일될 때 적용되므로(ROS-TCP-Connector Editor/ROSSettingsEditor.cs:66-93 과 같은 방식),
// 같은 실행 안에서는 커넥터가 ROS1 모드로 컴파일된 채 남는다.
//
// 근거 파일 줄 번호는 URDF-Importer v0.5.2 / ROS-TCP-Connector v0.7.0 패키지 소스 기준이다(단계 4 조사).
// 모든 출력은 "[SimSetup] " 로 시작한다. scripts/unity_batch.sh 가 이 줄만 발췌하고, "[SimSetup] FAIL" 이 있으면 실패로 본다.
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Xml.Linq;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Robotics.UrdfImporter;            // UrdfRobotExtensions, ImportSettings, UrdfRobot, UrdfJoint
using Unity.Robotics.UrdfImporter.Control;    // Controller, FKRobot  (JointControl 과 RuntimeUrdf 는 전역 네임스페이스)
using Unity.Robotics.ROSTCPConnector;         // ROSConnection
using Object = UnityEngine.Object;

public static class SimSetup
{
    // ── 경로·상수 ──────────────────────────────────────────────────────────────
    const string k_UrdfRelToAssets = "URDF/e0509/e0509.urdf";          // 단계 3 폴더를 통째로 복사한 곳
    const string k_PackageRoot     = "Assets/URDF/e0509";              // package://<pkg>/… 이 여기 아래로 풀린다
    const string k_ScenePath       = "Assets/Scenes/E0509Sim.unity";
    const string k_RosPrefabPath   = "Assets/Resources/ROSConnectionPrefab.prefab"; // ROSSettingsEditor.cs:57 의 리터럴 경로
    const string k_RosIp           = "127.0.0.1";
    const int    k_RosPort         = 10000;
    const string k_SimObjectName   = "Sim";
    const string k_TcpName         = "tcp";
    const string k_FlangeLinkName  = "rh_p12_rn_base";                 // CONTRACT §2-6: TCP 부착 기준 링크
    const float  k_TcpOffsetRosZ   = 0.236f;                            // CONTRACT §2-6: 툴 +Z 0.236 m

    // 드라이브 시작값 (단계 4). 단계 5에서 0.3° 수렴을 확인하며 조정한다.
    // Isaac 과 같은 값 (딸기 리포 strawberry_harvest/assets/robot/robot_assembly.usd @5710f09 의 drive:angular 덮어쓰기,
    // maxForce 는 configuration/robot_physics.usd = URDF effort). 감쇠/강성 비 0.1 → 시정수 0.1 s, 넘침 없음(단계 7 스윕).
    // 처음 시작값 1e5/1e3(비 0.01)은 한계 근처 계단 명령에서 J3 0.5°·J2 0.66° 넘침이 나서 버렸다(logs/m2/*_drive_sweep).
    static readonly Dictionary<string, (float k, float c, float f)> k_Drive = new Dictionary<string, (float, float, float)>
    {
        ["joint_1"] = (1e6f, 1e5f, 194f), ["joint_2"] = (1e6f, 1e5f, 194f), ["joint_3"] = (1e6f, 1e5f, 194f),
        ["joint_4"] = (1e6f, 1e5f, 66f),  ["joint_5"] = (1e6f, 1e5f, 66f),  ["joint_6"] = (1e6f, 1e5f, 66f),
        ["rh_p12_rn"] = (1e5f, 1e4f, 1000f), ["rh_r2"] = (1e5f, 1e4f, 1000f), ["rh_l1"] = (1e5f, 1e4f, 1000f), ["rh_l2"] = (1e5f, 1e4f, 1000f),
    };

    // 단계 5 런타임 스크립트. 아직 없으면 경고만 하고 넘어간다(단계 5 뒤 BuildScene 을 다시 돌리면 붙는다).
    // 단계 8 화면: GraspFollower(잡은 과실 따라가기·발행 제외), HarvestHud(Isaac 과 같은 HUD·그리퍼 카메라 창)
    static readonly string[] k_RuntimeComponents = { "JointStateMirror", "SimHud", "TcpTrail", "JointLimitMonitor", "JointStatePublisher", "StrawberryPublisher", "GraspFollower", "HarvestHud" };

    // 기대값 (CONTRACT §1-4 검증 결과). 어긋나면 임포트 축·스케일이 틀린 것이다.
    const int   k_ExpectedLinks = 15, k_ExpectedJoints = 14, k_ExpectedRevolute = 10, k_ExpectedBodies = 14;
    const float k_ExpectedFlangeHeight = 1.123f;   // 영점 자세 rh_p12_rn_base 높이 (ROS z → Unity y)
    const float k_ExpectedTcpHeight    = 1.359f;   // 영점 자세 TCP 높이

    // ── 로그 ───────────────────────────────────────────────────────────────────
    static void Log(string m)  => Debug.Log("[SimSetup] " + m);
    static void Warn(string m) => Debug.LogWarning("[SimSetup] WARN " + m);
    static void Fail(string m)
    {
        Debug.LogError("[SimSetup] FAIL " + m);
        if (Application.isBatchMode) EditorApplication.Exit(1);
        throw new InvalidOperationException("[SimSetup] FAIL " + m);
    }

    // ══════════════════════════════════════════════════════════════════════════
    // 1) ConfigureProject — ROS2 정의 심볼, ROSConnection 프리팹, 고정 시간 간격, 백그라운드 실행
    // ══════════════════════════════════════════════════════════════════════════
    [MenuItem("Sim/Configure Project")]
    public static void ConfigureProject()
    {
        try
        {
            Log($"ConfigureProject 시작  Unity {Application.unityVersion}  batch={Application.isBatchMode}  cwd={Directory.GetCurrentDirectory()}");
            ConfigureRos2Define();
            ConfigureRosConnectionPrefab();
            ConfigureTimeAndPlayer();
            SimEnvironment.ConfigureQuality();
            AssetDatabase.SaveAssets();
            Log("ConfigureProject OK. ROS2 정의는 다음 실행(재컴파일)부터 적용된다 → BuildScene 은 별도 실행.");
        }
        catch (Exception e) when (!(e is InvalidOperationException && e.Message.StartsWith("[SimSetup] FAIL")))
        {
            Fail("ConfigureProject 예외: " + e);
        }
    }

    // ROS-TCP-Connector 는 #if ROS2 로 프로토콜을 고른다(ROSConnection.cs:652-662, MessageSerializer/Deserializer).
    // ROSSettingsEditor.cs:78-93 은 활성 빌드 타깃 그룹에만 심볼을 쓴다. 여기서는 활성 그룹과 Standalone 둘 다 보장한다.
    static void ConfigureRos2Define()
    {
        var active = BuildPipeline.GetBuildTargetGroup(EditorUserBuildSettings.activeBuildTarget);
        var targets = new List<NamedBuildTarget> { NamedBuildTarget.Standalone };
        var activeNamed = NamedBuildTarget.FromBuildTargetGroup(active);
        if (!targets.Contains(activeNamed)) targets.Add(activeNamed);

        foreach (var t in targets)
        {
            string before = PlayerSettings.GetScriptingDefineSymbols(t);
            var defines = before.Split(';').Where(s => !string.IsNullOrWhiteSpace(s)).ToList();
            if (defines.Contains("ROS2"))
            {
                Log($"정의 심볼 [{t.TargetName}] 이미 ROS2 포함: '{before}'");
                continue;
            }
            defines.Add("ROS2");
            PlayerSettings.SetScriptingDefineSymbols(t, string.Join(";", defines));
            Log($"정의 심볼 [{t.TargetName}] '{before}' → '{PlayerSettings.GetScriptingDefineSymbols(t)}'");
        }
#if ROS2
        Log("이 컴파일(Assembly-CSharp-Editor)에서 ROS2 가 이미 활성");
#else
        Log("이 컴파일에서는 ROS2 가 아직 비활성 (정상: 다음 실행부터 적용)");
#endif
    }

    // ROSSettingsEditor.cs:45-62,99-104,143-146 이 하는 일을 EditorWindow 없이 재현한다.
    // 접속 기본값은 이미 127.0.0.1:10000 (ROSConnection.cs:22-59) 이지만 결정적으로 명시한다.
    static void ConfigureRosConnectionPrefab()
    {
        if (!AssetDatabase.IsValidFolder("Assets/Resources"))
            AssetDatabase.CreateFolder("Assets", "Resources");

        if (AssetDatabase.LoadAssetAtPath<GameObject>(k_RosPrefabPath) == null)
        {
            var go = new GameObject("ROSConnection");
            go.AddComponent<ROSConnection>();
            PrefabUtility.SaveAsPrefabAsset(go, k_RosPrefabPath);
            Object.DestroyImmediate(go);
            Log($"ROSConnection 프리팹 생성: {k_RosPrefabPath}");
        }

        GameObject root = PrefabUtility.LoadPrefabContents(k_RosPrefabPath);
        try
        {
            var ros = root.GetComponent<ROSConnection>();               // ?? 금지: UnityEngine.Object 의 가짜 null
            if (ros == null) ros = root.AddComponent<ROSConnection>();
            ros.RosIPAddress = k_RosIp;
            ros.RosPort = k_RosPort;
            ros.ConnectOnStart = true;        // Play 즉시 엔드포인트에 붙는다
            ros.ShowHud = false;              // 커넥터 자체 HUD(좌상단) 끔 — 좌상단은 Isaac 과 같은 HarvestHud 자리. 연결 상태는 SimHud(I 키) 첫 줄
            ros.listenForTFMessages = false;  // /tf 자동 구독 안 함 (ROSConnection.cs:57-59,486-487)
            PrefabUtility.SaveAsPrefabAsset(root, k_RosPrefabPath);
        }
        finally { PrefabUtility.UnloadPrefabContents(root); }

        // HUD 의 Connect 버튼은 PlayerPrefs 값을 쓴다(ROSConnection.cs:1081-1087). 같은 값으로 맞춘다.
        ROSConnection.SetIPPref(k_RosIp);
        ROSConnection.SetPortPref(k_RosPort);
        PlayerPrefs.Save();                   // batchmode -quit 에서도 디스크에 남게
        Log($"ROSConnection 프리팹: {k_RosIp}:{k_RosPort}, ConnectOnStart, 커넥터 HUD 끔, TF 구독 끔");
    }

    // 물리 스텝 = Isaac 과 같은 60 Hz (CONTRACT §9-1). 에디터 창이 포커스를 잃어도 돌게 runInBackground.
    static void ConfigureTimeAndPlayer()
    {
        var tm = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TimeManager.asset").FirstOrDefault();
        if (tm == null) { Warn("TimeManager.asset 을 못 열어 Fixed Timestep 을 그대로 둔다"); }
        else
        {
            var so = new SerializedObject(tm);
            var p = so.FindProperty("Fixed Timestep");
            if (p == null) Warn("TimeManager 'Fixed Timestep' 속성 없음");
            else
            {
                float before = p.floatValue;
                p.floatValue = 1f / 60f;
                so.ApplyModifiedPropertiesWithoutUndo();
                Log($"Fixed Timestep {before:F5} → {p.floatValue:F5} s (60 Hz)");
            }
        }
        PlayerSettings.runInBackground = true;
        Log("PlayerSettings.runInBackground = true");

        // 물리 풀이기 = Isaac 과 같은 TGS (robot_physics.usd /physicsScene physxScene:solverType = "TGS").
        // 관절 반복 횟수(32/1)는 BuildScene 이 루트 ArticulationBody 에 건다.
        var dm = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/DynamicsManager.asset").FirstOrDefault();
        if (dm == null) Warn("DynamicsManager.asset 을 못 열어 풀이기를 그대로 둔다");
        else
        {
            var so = new SerializedObject(dm);
            var st = so.FindProperty("m_SolverType");
            if (st == null) Warn("DynamicsManager 'm_SolverType' 속성 없음");
            else
            {
                int before = st.intValue;
                st.intValue = 1;   // 0 = PGS, 1 = TGS
                so.ApplyModifiedPropertiesWithoutUndo();
                Log($"물리 풀이기 {(before == 1 ? "TGS" : "PGS")} → TGS (Isaac 과 같게)");
            }
        }
    }

    // ══════════════════════════════════════════════════════════════════════════
    // 2) BuildScene — URDF 임포트(없을 때만) → 후처리 → Sim 오브젝트 → 카메라·조명 → 씬 저장 → Build Settings
    // ══════════════════════════════════════════════════════════════════════════
    [MenuItem("Sim/Build Scene")]
    public static void BuildScene()
    {
        try
        {
            Log($"BuildScene 시작  Unity {Application.unityVersion}  batch={Application.isBatchMode}  nographics={SystemInfo.graphicsDeviceType == UnityEngine.Rendering.GraphicsDeviceType.Null}  cwd={Directory.GetCurrentDirectory()}");
#if ROS2
            Log("ROS2 정의 활성 (커넥터가 ROS2 프로토콜로 컴파일됨)");
#else
            Warn("ROS2 정의 비활성 — ConfigureProject 를 먼저 별도 실행해야 한다. 이 상태로 Play 하면 ROS1 프로토콜로 통신한다.");
#endif
            EnsureCwdIsProjectRoot();

            Scene scene = OpenOrCreateScene();
            UrdfRobot existing = Object.FindObjectOfType<UrdfRobot>();
            // 규칙 1(있으면 생략) 때문에 URDF 가 바뀌어도 재임포트되지 않는다. SIM_FORCE_REIMPORT=1 이면 기존 로봇을 지우고 다시 임포트한다.
            // (.dae 만 바뀐 경우는 시각이 프리팹 인스턴스라 에셋 재임포트로 자동 반영되므로 필요 없다.)
            if (existing != null && Environment.GetEnvironmentVariable("SIM_FORCE_REIMPORT") == "1")
            {
                Warn($"SIM_FORCE_REIMPORT=1: 기존 UrdfRobot '{existing.gameObject.name}' 을 지우고 다시 임포트한다");
                Object.DestroyImmediate(existing.gameObject);
                existing = null;
            }
            GameObject robot;
            if (existing != null)
            {
                robot = existing.gameObject;
                Log($"씬에 UrdfRobot '{robot.name}' 이 이미 있어 임포트를 생략한다 (규칙 1, 강제는 SIM_FORCE_REIMPORT=1)");
            }
            else
            {
                robot = ImportUrdf();
            }

            PostProcessRobot(robot);
            ChooseVisualAxis(robot);
            VerifyRobot(robot);
            Transform tcp = EnsureTcpChild(robot);
            Bounds bounds = ComputeRendererBounds(robot);
            EnsureSimObject();
            EnsureCameraAndLight(bounds);
            // Isaac 씬 정적 형상·조명·재질·카메라 프리셋 (assets_src/scene/scene.json 이 있을 때)
            if (File.Exists(Path.Combine(Application.dataPath, "..", "..", "assets_src", "scene", "scene.json"))) SimEnvironment.Build(robot);
            else Warn("assets_src/scene/scene.json 없음 — 환경 없이 진행 (python3 -B scripts/export_isaac_scene.py)");

            // 패키지는 임포트 경로에서 SaveAssets 를 부르지 않는다(.mat·STL .prefab/.asset·TagManager 가 메모리에만 있을 수 있다).
            AssetDatabase.SaveAssets();
            if (!EditorSceneManager.SaveScene(scene, k_ScenePath)) Fail("씬 저장 실패: " + k_ScenePath);
            EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(k_ScenePath, true) };
            AssetDatabase.SaveAssets();
            Log($"BuildScene OK → {k_ScenePath} (Build Settings 등록). TCP 자식 '{tcp.name}' @ {k_FlangeLinkName}");
        }
        catch (Exception e) when (!(e is InvalidOperationException && e.Message.StartsWith("[SimSetup] FAIL")))
        {
            Fail("BuildScene 예외: " + e);
        }
    }

    // StlImporter/StlReader 가 "Assets/…" 상대 경로를 StreamReader 로 직접 연다(StlImporter.cs:38, StlReader.cs:31).
    static void EnsureCwdIsProjectRoot()
    {
        string projectRoot = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
        string cwd = Path.GetFullPath(Directory.GetCurrentDirectory());
        if (!string.Equals(cwd.TrimEnd('/'), projectRoot.TrimEnd('/'), StringComparison.Ordinal))
        {
            Directory.SetCurrentDirectory(projectRoot);
            Warn($"cwd 를 프로젝트 루트로 바꿈: {cwd} → {projectRoot}");
        }
    }

    static Scene OpenOrCreateScene()
    {
        if (File.Exists(k_ScenePath))
        {
            Log("기존 씬 열기: " + k_ScenePath);
            return EditorSceneManager.OpenScene(k_ScenePath, OpenSceneMode.Single);
        }
        if (!AssetDatabase.IsValidFolder("Assets/Scenes")) AssetDatabase.CreateFolder("Assets", "Scenes");
        Log("새 빈 씬 생성 (카메라·조명은 직접 배치)");
        return EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
    }

    // ── URDF 임포트 ────────────────────────────────────────────────────────────
    static GameObject ImportUrdf()
    {
        // 절대 경로여야 UrdfAssetPathHandler.GetRelativeAssetPath 가 Application.dataPath 분기를 타서
        // packageRoot = "Assets/URDF/e0509" 가 된다(UrdfAssetPathHandler.cs:53-74, UrdfRobotExtensions.cs:115).
        string urdfAbs = Path.Combine(Application.dataPath, k_UrdfRelToAssets);
        if (!File.Exists(urdfAbs)) Fail("URDF 없음: " + urdfAbs);

        // 메쉬가 AssetDatabase 에 들어 있어야 한다. 하나라도 LoadAssetAtPath 가 null 이면 임포터가
        // EditorUtility.DisplayDialogComplex / OpenFilePanel 을 띄운다(LocateAssetHandler.cs:45-110) → batchmode 에서 치명적.
        AssetDatabase.Refresh(ImportAssetOptions.ForceSynchronousImport);
        PreflightMeshes(urdfAbs);

        // 재질은 Shader.Find("Standard") 로 만든다(MaterialExtensions.cs:99-153). -nographics 에서 null 이면 UrdfMaterial.cs:76 에서 NRE.
        if (Shader.Find("Standard") == null)
            Fail("Shader.Find(\"Standard\") == null. -nographics 를 빼고 다시 실행");

        var settings = new ImportSettings
        {
            // yAxis = 추가 회전 없음. .dae 는 ColladaAssetPostProcessor 가 Z_UP 을, .stl 은 StlReader 가 Ros2Unity 를 이미 적용한다.
            // 처음 계획의 Z-axis 는 이 메쉬들에 틀린 가정이었다(단계 4 기록, CONTRACT §9-3). ChooseVisualAxis 가 뒤에서 다시 확인한다.
            chosenAxis   = ImportSettings.axisType.yAxis,
            convexMethod = ImportSettings.convexDecomposer.unity,  // 가벼운 쪽: MeshFilter 당 convex MeshCollider 하나, 네이티브 VHACD·.asset 생성 없음
            OverwriteExistingPrefabs = false,                      // STL 의 <name>.prefab/_0.asset 재사용
        };

        RuntimeUrdf.SetRuntimeMode(false);   // 에디터 경로(AssetDatabase·PrefabUtility). CreateRuntime 은 .dae 콜라이더를 못 만든다
        Selection.activeObject = null;       // ImportPipelinePostCreate 가 Selection.activeObject 아래에 부모를 잡는다(UrdfRobotExtensions.cs:148)

        Log($"URDF 임포트 시작: {urdfAbs}  axis={settings.chosenAxis} convex={settings.convexMethod}");
        GameObject robot = null;
        // loadStatus=false 면 첫 MoveNext() 안에서 전부 끝나고 마지막에 로봇을 한 번 yield 한다(UrdfRobotExtensions.cs:175-196).
        // Init 실패(IsValidAssetPath) 땐 yield break → Current 가 null.
        IEnumerator<GameObject> it = UrdfRobotExtensions.Create(urdfAbs, settings, loadStatus: false, forceRuntimeMode: false);
        try
        {
            while (it.MoveNext())
                if (it.Current != null) robot = it.Current;
        }
        finally { it.Dispose(); }

        if (robot == null) Fail("임포터가 로봇을 반환하지 않았다 (로그의 'URDF file and resources must be placed in project folder' 또는 예외 확인)");
        if (settings.linksLoaded != settings.totalLinks || settings.totalLinks != k_ExpectedLinks)
            Warn($"링크 로드 {settings.linksLoaded}/{settings.totalLinks} (기대 {k_ExpectedLinks})");
        Log($"임포트 완료: '{robot.name}' 링크 {settings.linksLoaded}/{settings.totalLinks}");
        return robot;
    }

    // URDF 가 참조하는 모든 메쉬를 임포터와 같은 규칙으로 풀어서 미리 확인한다(UrdfAssetPathHandler.cs:90-128).
    static void PreflightMeshes(string urdfAbs)
    {
        var files = XDocument.Load(urdfAbs).Descendants("mesh")
            .Select(m => (string)m.Attribute("filename")).Where(f => f != null).Distinct().ToList();
        var problems = new List<string>();
        foreach (string f in files)
        {
            if (!f.StartsWith("package://")) { problems.Add("package:// 아님: " + f); continue; }
            string assetPath = k_PackageRoot + "/" + f.Substring("package://".Length);
            if (!File.Exists(assetPath)) { problems.Add("파일 없음: " + assetPath); continue; }
            // .stl 은 임포트 중에 <name>.prefab 으로 변환된다(StlAssetPostProcessor). .dae 는 ModelImporter 결과가 있어야 한다.
            if (assetPath.EndsWith(".dae", StringComparison.OrdinalIgnoreCase) &&
                AssetDatabase.LoadAssetAtPath<GameObject>(assetPath) == null)
                problems.Add("Collada 가 모델로 임포트되지 않음: " + assetPath);
        }
        if (problems.Count > 0) Fail($"메쉬 사전 점검 실패 {problems.Count}건:\n  " + string.Join("\n  ", problems));
        Log($"메쉬 사전 점검 통과: {files.Count}개 (.dae {files.Count(x => x.EndsWith(".dae"))}, .stl {files.Count(x => x.EndsWith(".stl"))})");
    }

    // ── 후처리 (여러 번 돌려도 같은 결과) ──────────────────────────────────────
    static void PostProcessRobot(GameObject robot)
    {
        // 임포터가 루트에 붙이는 Controller 는 Play 때 FKRobot·JointControl 을 만들고 모든 드라이브의 forceLimit 을 0 으로,
        // jointFriction·angularDamping 을 10 으로 덮어쓴다(Controller.cs:33-50). 에디터에서 지우면 그 일이 생기지 않는다.
        // UrdfRobot.AddController()/SetRigidbodiesUseGravity() 는 정적 플래그 토글이라 쓰지 않는다(UrdfRobot.cs:31-62,87-115).
        int removed = 0;
        foreach (var c in robot.GetComponentsInChildren<Controller>(true))   { Object.DestroyImmediate(c); removed++; }
        foreach (var c in robot.GetComponentsInChildren<FKRobot>(true))      { Object.DestroyImmediate(c); removed++; }
        foreach (var c in robot.GetComponentsInChildren<JointControl>(true)) { Object.DestroyImmediate(c); removed++; }

        var bodies = robot.GetComponentsInChildren<ArticulationBody>(true);
        int roots = 0;
        foreach (var ab in bodies)
        {
            ab.useGravity = false;                  // 중력 off (D-4)
            if (ab.isRoot)                          // 루트 AB 는 base_link (world 링크엔 AB 가 없다)
            {
                ab.immovable = true; roots++;
                // 관절 풀이 반복(Isaac 32/1)은 씬에 직렬화되지 않아 JointStateMirror.Awake 가 Play 때마다 건다.
            }
            if (ab.jointType == ArticulationJointType.RevoluteJoint || ab.jointType == ArticulationJointType.PrismaticJoint)
            {
                // 임포터는 lower/upper(deg)·forceLimit(=effort)·maxAngularVelocity(=velocity) 만 넣고 강성·감쇠는 0 으로 둔다
                // (UrdfJointRevolute.cs:165-175). ArticulationDrive 는 struct 라 읽고 고쳐 다시 넣는다.
                ArticulationDrive d = ab.xDrive;
                var uj = ab.GetComponent<UrdfJoint>();
                string jn = uj != null ? uj.jointName : ab.name;
                if (!k_Drive.TryGetValue(jn, out var g)) { Fail($"드라이브 값 표에 없는 관절: {jn}"); continue; }
                d.stiffness  = g.k;
                d.damping    = g.c;
                d.forceLimit = g.f;
                ab.xDrive = d;
                // maxAngularVelocity 는 임포터가 URDF 속도 한계(rad/s)로 넣은 값을 그대로 둔다. 몸체 각속도 상한이라
                // 관절 속도 한계와 같지 않지만, 풀면 원본보다 관대해진다. 속도 위반은 JointLimitMonitor 가 따로 센다(CONTRACT §2-5-5).
            }
        }
        int colliders = 0;
        foreach (var col in robot.GetComponentsInChildren<Collider>(true)) { col.enabled = false; colliders++; }   // 접촉은 다루지 않는다

        Log($"후처리: 컨트롤 컴포넌트 제거 {removed}, ArticulationBody {bodies.Length}(루트 {roots}, immovable·useGravity=false, 드라이브 = Isaac 값(팔 1e6/1e5/194·66, 그리퍼 1e5/1e4/1000)), Collider 비활성 {colliders}");
    }

    // ── 시각 메쉬 축 판정 ──────────────────────────────────────────────────────
    // chosenAxis 는 링크 프레임(관절 origin 변환, Ros2Unity)에는 영향이 없고 Visual/Collision 래퍼의 localRotation 에만
    // Euler(-90,0,90) 을 곱한다(UrdfRobotExtensions.CorrectAxis, 224-266). .dae 는 ColladaAssetPostProcessor 가 Z_UP 을,
    // .stl 은 StlReader 가 Ros2Unity 를 이미 적용하므로 어느 쪽이 맞는지는 소스만으로 못 정한다(단계 4 조사 미해결 항목).
    // 기구학으로 판정한다: 영점 자세에서 각 링크의 시각 bounds 가 그 링크에 달린 자식 관절 원점을 포함해야 한다.
    // zAxis·yAxis 둘 다 적용해 보고 포함 개수가 많은(동률이면 수평 폭이 작은) 쪽을 택해 남긴다. 여러 번 돌려도 결과가 같다.
    static void ChooseVisualAxis(GameObject robot)
    {
        var ur = robot.GetComponent<UrdfRobot>();
        if (ur == null) { Warn("UrdfRobot 컴포넌트가 없어 축 판정을 건너뛴다"); return; }
        var candidates = new[] { ImportSettings.axisType.zAxis, ImportSettings.axisType.yAxis };
        var scores = new List<(ImportSettings.axisType axis, int inside, int total, Bounds all)>();
        foreach (var axis in candidates)
        {
            ur.SetAxis(axis);
            UrdfRobotExtensions.CorrectAxis(robot);      // currentOrientation != chosenAxis 일 때만 회전을 곱한다
            Physics.SyncTransforms();
            var s = ScoreVisualAxis(robot, axis);
            scores.Add((axis, s.inside, s.total, s.all));
            Log($"축 후보 {axis}: 자식 관절 원점을 포함하는 링크 {s.inside}/{s.total}, 전체 bounds size={s.all.size.ToString("F3")} center={s.all.center.ToString("F3")}");
        }
        var best = scores.OrderByDescending(s => s.inside).ThenBy(s => Mathf.Max(s.all.size.x, s.all.size.z)).First();
        ur.SetAxis(best.axis);
        UrdfRobotExtensions.CorrectAxis(robot);
        Physics.SyncTransforms();
        Log($"시각 메쉬 축 = {best.axis} 선택 (포함 {best.inside}/{best.total}). 처음 계획의 'Z-axis' 와 다르면 PROGRESS 에 근거를 적는다.");
        if (best.inside < best.total) Warn($"어느 축에서도 모든 링크가 자식 관절 원점을 포함하지는 않는다 ({best.inside}/{best.total}) — 에디터에서 눈으로 확인(H3)");
    }

    static (int inside, int total, Bounds all) ScoreVisualAxis(GameObject robot, ImportSettings.axisType axis)
    {
        int inside = 0, total = 0;
        foreach (var j in robot.GetComponentsInChildren<UrdfJoint>(true))
        {
            Transform parentLink = j.transform.parent;           // 관절 컴포넌트는 자식 링크에 있고, 그 부모가 부모 링크
            if (parentLink == null || parentLink.GetComponent<UrdfLink>() == null) continue;
            Transform visuals = parentLink.Find("Visuals");
            var rends = visuals != null ? visuals.GetComponentsInChildren<Renderer>(true) : new Renderer[0];
            if (rends.Length == 0) continue;
            Bounds b = rends[0].bounds;
            foreach (var r in rends) b.Encapsulate(r.bounds);
            b.Expand(0.06f);                                      // 메쉬 두께·플랜지 여유 3 cm
            bool hit = b.Contains(j.transform.position);          // 자식 링크 transform 위치 = 관절 origin
            total++; if (hit) inside++;
            Log($"    [{axis}] {parentLink.name,-16} visual bounds c={b.center.ToString("F3")} s={b.size.ToString("F3")} ∋ {j.jointName}@{j.transform.position.ToString("F3")} → {(hit ? "O" : "X")}");
        }
        var allR = robot.GetComponentsInChildren<Renderer>(true);
        Bounds all = allR.Length > 0 ? allR[0].bounds : new Bounds();
        foreach (var r in allR) all.Encapsulate(r.bounds);
        return (inside, total, all);
    }

    // 개수·한계·영점 FK 를 로그로 남긴다. CONTRACT §9-3 의 열린 질문(±360° 한계 전달, DAE 스케일, world 루트 처리)에 답하는 근거.
    static void VerifyRobot(GameObject robot)
    {
        var links  = robot.GetComponentsInChildren<UrdfLink>(true);
        var joints = robot.GetComponentsInChildren<UrdfJoint>(true);
        var bodies = robot.GetComponentsInChildren<ArticulationBody>(true);
        var revolute = joints.Where(j => j.JointType == UrdfJoint.JointTypes.Revolute).ToList();
        Log($"검증: 링크 {links.Length}(기대 {k_ExpectedLinks}) 관절 {joints.Length}(기대 {k_ExpectedJoints}) 회전관절 {revolute.Count}(기대 {k_ExpectedRevolute}) AB {bodies.Length}(기대 {k_ExpectedBodies})");
        if (links.Length != k_ExpectedLinks || joints.Length != k_ExpectedJoints || revolute.Count != k_ExpectedRevolute || bodies.Length != k_ExpectedBodies)
            Fail("링크·관절·ArticulationBody 개수가 기대와 다르다");

        // 관절 표 — 이름(링크) 타입 한계 forceLimit maxAngVel 강성/감쇠 anchor 축
        foreach (var j in joints.OrderBy(j => j.transform.GetSiblingIndex()).OrderBy(j => Depth(j.transform)))
        {
            var ab = j.GetComponent<ArticulationBody>();
            if (ab == null) { Warn($"  {j.jointName}: ArticulationBody 없음"); continue; }
            var d = ab.xDrive;
            string lim = ab.jointType == ArticulationJointType.RevoluteJoint
                ? $"[{d.lowerLimit:F4}, {d.upperLimit:F4}]° force={d.forceLimit:G4} maxAngVel={ab.maxAngularVelocity:F4} k={d.stiffness:G} c={d.damping:G} axis(anchor X)={(ab.anchorRotation * Vector3.right).ToString("F3")}"
                : "";
            Log($"  joint {j.jointName,-26} {ab.jointType,-14} link={j.gameObject.name,-20} isRoot={ab.isRoot,-5} immovable={ab.immovable,-5} gravity={ab.useGravity,-5} damp={ab.angularDamping:F2} {lim}");
        }

        // 영점 FK: rh_p12_rn_base 높이 1.123 m (CONTRACT §1-4). 틀리면 축·스케일 문제.
        Transform flange = FindDeep(robot.transform, k_FlangeLinkName);
        if (flange == null) Fail($"링크 '{k_FlangeLinkName}' 없음");
        Vector3 p = flange.position;
        bool okFlange = Mathf.Abs(p.y - k_ExpectedFlangeHeight) < 2e-3f && p.magnitude - Mathf.Abs(p.y) < 2e-3f;
        Log($"영점 FK: {k_FlangeLinkName} world={p.ToString("F4")} (기대 Unity y≈{k_ExpectedFlangeHeight}) → {(okFlange ? "OK" : "MISMATCH")}");
        if (!okFlange) Warn("플랜지 영점 위치가 기대와 다르다. 관절 origin 변환 또는 축 설정 확인");

        Transform world = FindDeep(robot.transform, "world");
        Log($"루트 링크 'world': {(world == null ? "없음" : (world.GetComponent<ArticulationBody>() == null ? "AB 없음(기대)" : "AB 있음"))}; 루트 AB = {bodies.FirstOrDefault(b => b.isRoot)?.gameObject.name}");
    }

    static int Depth(Transform t) { int d = 0; while (t.parent != null) { t = t.parent; d++; } return d; }

    static Transform FindDeep(Transform root, string name)
    {
        if (root.name == name) return root;
        foreach (Transform c in root) { var r = FindDeep(c, name); if (r != null) return r; }
        return null;
    }

    // TCP 빈 자식: ROS 로컬 (0, 0, +0.236) = Unity 로컬 (0, 0.236, 0). 임포터의 Ros2Unity 는 (x,y,z)→(-y,z,x) (BuiltInExtensions.cs:103-106).
    static Transform EnsureTcpChild(GameObject robot)
    {
        Transform flange = FindDeep(robot.transform, k_FlangeLinkName);
        Transform tcp = flange.Find(k_TcpName);
        if (tcp == null)
        {
            tcp = new GameObject(k_TcpName).transform;
            tcp.SetParent(flange, false);
        }
        tcp.localPosition = new Vector3(0f, k_TcpOffsetRosZ, 0f);
        tcp.localRotation = Quaternion.identity;
        tcp.localScale = Vector3.one;
        Vector3 p = tcp.position;
        bool ok = Mathf.Abs(p.y - k_ExpectedTcpHeight) < 2e-3f;
        Log($"TCP 영점 world={p.ToString("F4")} (기대 y≈{k_ExpectedTcpHeight}) → {(ok ? "OK" : "MISMATCH")}");
        return tcp;
    }

    static Bounds ComputeRendererBounds(GameObject robot)
    {
        var renderers = robot.GetComponentsInChildren<Renderer>(true);
        if (renderers.Length == 0) { Warn("Renderer 가 없다 (visual 메쉬 미로딩?)"); return new Bounds(Vector3.up * 0.6f, Vector3.one); }
        Bounds b = renderers[0].bounds;
        foreach (var r in renderers) b.Encapsulate(r.bounds);
        // 메쉬 정점은 mm, URDF scale 0.001 → 전체 높이 1.2–1.5 m 가 정상. 수백 m 면 스케일이 두 번 적용된 것.
        Log($"Renderer {renderers.Length}개, 전체 bounds center={b.center.ToString("F3")} size={b.size.ToString("F3")} (정상: 높이 ≈1.2–1.5 m)");
        if (b.size.y > 5f || b.size.y < 0.5f) Warn("로봇 bounds 높이가 비정상 — 메쉬 스케일/축 확인");
        return b;
    }

    // 'Sim' 빈 오브젝트에 단계 5 컴포넌트를 이름으로 붙인다(없는 타입은 경고). 이미 붙어 있으면 건너뛴다.
    static void EnsureSimObject()
    {
        GameObject sim = GameObject.Find(k_SimObjectName);
        if (sim == null) sim = new GameObject(k_SimObjectName);
        var attached = new List<string>(); var missing = new List<string>(); var present = new List<string>();
        foreach (string typeName in k_RuntimeComponents)
        {
            Type t = FindMonoBehaviourType(typeName);
            if (t == null) { missing.Add(typeName); continue; }
            if (sim.GetComponent(t) != null) { present.Add(typeName); continue; }
            sim.AddComponent(t); attached.Add(typeName);
        }
        // 씬 기본 모드는 Mirror(M1). 에디터에서 Command 로 바꿔 저장했어도 BuildScene 이 되돌린다(HUMAN_STEPS H3-2).
        var mirrorType = FindMonoBehaviourType("JointStateMirror");
        var mirror = mirrorType != null ? sim.GetComponent(mirrorType) : null;
        if (mirror != null)
        {
            var so = new SerializedObject(mirror);
            var mp = so.FindProperty("mode");
            if (mp != null && mp.enumValueIndex != 0) { Log($"JointStateMirror.mode {mp.enumNames[mp.enumValueIndex]} → Mirror (씬 기본값)"); mp.enumValueIndex = 0; so.ApplyModifiedPropertiesWithoutUndo(); }
        }
        Log($"'{k_SimObjectName}' 컴포넌트: 새로 붙임 [{string.Join(", ", attached)}] 이미 있음 [{string.Join(", ", present)}] 타입 없음 [{string.Join(", ", missing)}]");
        if (missing.Count > 0) Warn("단계 5 스크립트가 아직 없다. Assets/Scripts 에 추가한 뒤 BuildScene 을 다시 실행하면 붙는다.");
    }

    static Type FindMonoBehaviourType(string name)
    {
        foreach (var asm in AppDomain.CurrentDomain.GetAssemblies())
        {
            Type[] types;
            try { types = asm.GetTypes(); }
            catch (ReflectionTypeLoadException e) { types = e.Types.Where(t => t != null).ToArray(); }
            var hit = types.FirstOrDefault(t => t.Name == name && typeof(MonoBehaviour).IsAssignableFrom(t));
            if (hit != null) return hit;
        }
        return null;
    }

    // 카메라: 로봇 전체가 보이게 bounds 를 보고 앞·오른쪽·위에서 내려다본다. 조명: 방향광 하나.
    static void EnsureCameraAndLight(Bounds bounds)
    {
        Camera cam = Object.FindObjectOfType<Camera>();
        if (cam == null)
        {
            var go = new GameObject("Main Camera");
            go.tag = "MainCamera";
            cam = go.AddComponent<Camera>();
            go.AddComponent<AudioListener>();
        }
        // 작업 반경(≈0.9 m)과 팔을 뻗은 자세까지 들어오게 중심은 bounds 중심, 거리는 약 2.8 m
        Vector3 center = new Vector3(0f, Mathf.Max(0.6f, bounds.center.y), 0f);
        cam.transform.position = center + new Vector3(1.7f, 0.9f, -2.1f);
        cam.transform.LookAt(center);
        cam.fieldOfView = 50f;
        cam.nearClipPlane = 0.05f;
        cam.farClipPlane = 50f;
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.backgroundColor = new Color(0.16f, 0.17f, 0.20f);

        Light light = Object.FindObjectsOfType<Light>().FirstOrDefault(l => l.type == LightType.Directional);
        if (light == null)
        {
            var go = new GameObject("Directional Light");
            light = go.AddComponent<Light>();
            light.type = LightType.Directional;
        }
        light.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
        light.intensity = 1.0f;
        light.shadows = LightShadows.Soft;
        Log($"카메라 {cam.transform.position.ToString("F2")} → {center.ToString("F2")}, 방향광 (50,-30,0)");
    }

    // ══════════════════════════════════════════════════════════════════════════
    // 3) OpenSceneAndPlay — 에디터(그래픽 모드)를 터미널에서 띄울 때 씬을 열고 Play 까지 누른다 (scripts/open_editor.sh).
    //    모드는 씬을 고치지 않고 명령줄 -simMode mirror|command 로 정한다(JointStateMirror.ApplyModeOverride).
    //    그래서 에디터에서 씬을 저장할 일이 없다. -simNoPlay 를 주면 씬만 열고 Play 는 누르지 않는다.
    // ══════════════════════════════════════════════════════════════════════════
    public static void OpenSceneAndPlay()
    {
        if (Application.isBatchMode) { Warn("OpenSceneAndPlay 는 그래픽 모드 에디터용이다 (batchmode 에서는 아무것도 안 함)"); return; }
        var args = Environment.GetCommandLineArgs();
        bool play = !args.Contains("-simNoPlay");
        string mode = "mirror";
        for (int i = 0; i + 1 < args.Length; i++) if (args[i] == "-simMode") mode = args[i + 1];
        // 프로젝트 로드 직후에는 에디터가 아직 준비 중일 수 있어 한 박자 늦춘다
        EditorApplication.delayCall += () =>
        {
            if (!File.Exists(k_ScenePath)) { Debug.LogError("[SimSetup] 씬 없음: " + k_ScenePath + " — 먼저 BuildScene"); return; }
            EditorSceneManager.OpenScene(k_ScenePath, OpenSceneMode.Single);
            Log($"씬 열기: {k_ScenePath} (모드 {mode}, Play {(play ? "자동" : "안 함")})");
            // Play 는 같은 호출에서 요청한다. delayCall 로 한 번 더 미루면 에디터 창이 포커스를 받아 다음 틱이 돌 때까지
            // 수십 초–몇 분 멈췄다(10-25, 포커스 없이 띄운 mirror 4회 모두 20 s–4분 이상)
            if (play) EditorApplication.EnterPlaymode();
        };
    }
}
