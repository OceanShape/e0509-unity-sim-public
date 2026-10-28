// 단계 8 화면 — Isaac 씬의 정적 형상을 같은 배치로 가져와 Environment 를 만들고, 로봇 재질·화질·카메라 프리셋을 정한다.
// 입력: ../assets_src/scene/scene.json + textures (scripts/export_isaac_scene.py 가 딸기 리포 USD 에서 읽어 만든 것)
// SimSetup.BuildScene 이 Build() 를, ConfigureProject 가 ConfigureQuality() 를 부른다. 여러 번 돌려도 같은 결과(Environment 를 지우고 다시 만든다).
// 환경 오브젝트에는 콜라이더를 두지 않는다(접촉은 다루지 않는다). 과실 좌표 발행(StrawberryPublisher)은 고정 목록에서 붙은·수확된 과실을 뺀 것이고 메쉬에서 읽지 않는다.
// 단계 8 화면(10-21): 보드 하이라이트 테두리 20장(꺼 둔 채로) · 보드 방위 글자 텍스처 9장(BoardDisplay) · HUD 라벨 PNG(Resources/HudLabels) ·
//   그리퍼 D455 본체(rh_p12_rn_base 자식, 레이어 30) · 손목 컬러 카메라(WristCamera) · 카메라 프리셋 렌즈(초점거리, 가로 맞춤).
//
// RenderPreview(): 그래픽 모드 batchmode(-nographics 없이)에서 카메라 프리셋마다 PNG 를 logs/render/ 에 남긴다(사람 없이 화면 확인용).
//   bash scripts/unity_batch.sh 15 render -executeMethod SimEnvironment.RenderPreview   (UNITY_GRAPHICS=1)
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Unity.Robotics.UrdfImporter;
using Object = UnityEngine.Object;

public static class SimEnvironment
{
    // ── scene.json 구조 (JsonUtility) ──
    [Serializable] class SceneJ { public MeshJ[] meshes; public MatJ[] materials; public ObjJ[] objects; public BoxJ[] boxes; public LightJ[] lights; public CamJ[] cameras; public string checks;
                                  public HlJ[] highlights; public GlowJ glow; public LabJ[] board_label_textures; public WristJ wrist_camera; }
    [Serializable] class HlJ { public string name; public string mesh; }
    [Serializable] class GlowJ { public float[] color; public float intensity; }
    [Serializable] class LabJ { public string key; public string file; }
    [Serializable] class WristJ { public string parent; public float[] pos; public float[] forward; public float[] up; public float vfov_deg; public float hfov_deg; public int[] res; }
    [Serializable] class MeshJ { public string name; public float[] positions; public float[] normals; public float[] uvs; public int triangles; }
    [Serializable] class MatJ { public string name; public float[] color; public float smoothness; public float metallic; public string albedo; public string normal; }
    [Serializable] class ObjJ { public string name; public string mesh; public string material; public bool ripe; public string parent; public float[] pos; public float[] rot; public float[] scale; }
    [Serializable] class BoxJ { public string name; public string material; public float[] pos; public float[] rot; public float[] scale; }
    [Serializable] class LightJ { public string name; public string type; public float[] pos; public float[] forward; public float[] color; public float intensity; public float width; public float height; }
    [Serializable] class CamJ { public string name; public float[] pos; public float[] forward; public float[] up; public float vfov_deg; public float focal_mm; public float h_aperture_mm; }

    const string k_Root = "Environment";
    const string k_GenDir = "Assets/Environment";
    const string k_HudLabelDir = "Assets/Resources/HudLabels";     // HarvestHud 가 Resources.Load("HudLabels/<이름>")
    public const int k_BodyLayer = 30;                              // 그리퍼 D455 본체 — 손목 카메라에서 뺀다(렌즈 앞을 가린다)
    const float k_RobotCloseFocal = 16.2f;                          // 로봇 근접 구도: 16:9 에서 세로 40° 였던 화면(가로 65.8°)을 같은 가로 화각으로
    static string ProjectRoot => Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
    static string SimRoot => Path.GetFullPath(Path.Combine(Application.dataPath, "..", ".."));
    static string SceneJson => Path.Combine(SimRoot, "assets_src", "scene", "scene.json");

    static void Log(string m) => Debug.Log("[SimSetup] " + m);

    static Vector3 V(float[] a) => new Vector3(a[0], a[1], a[2]);
    static Quaternion Q(float[] a) => new Quaternion(a[0], a[1], a[2], a[3]);
    static Color Col(float[] a) => new Color(a[0], a[1], a[2], 1f);

    static SceneJ Load()
    {
        if (!File.Exists(SceneJson)) throw new FileNotFoundException("scene.json 없음 — 먼저 python3 -B scripts/export_isaac_scene.py", SceneJson);
        return JsonUtility.FromJson<SceneJ>(File.ReadAllText(SceneJson));
    }

    static void EnsureFolder(string path)
    {
        if (AssetDatabase.IsValidFolder(path)) return;
        string parent = Path.GetDirectoryName(path).Replace('\\', '/');
        EnsureFolder(parent);
        AssetDatabase.CreateFolder(parent, Path.GetFileName(path));
    }

    // ══════════════════════════════════════════════════════════════════════
    public static void Build(GameObject robot)
    {
        var s = Load();
        EnsureFolder(k_GenDir + "/Meshes"); EnsureFolder(k_GenDir + "/Materials"); EnsureFolder(k_GenDir + "/Textures");

        // 텍스처 복사·임포트
        var texByFile = new Dictionary<string, Texture2D>();
        foreach (var m in s.materials)
            foreach (var (file, isNormal) in new[] { (m.albedo, false), (m.normal, true) })
            {
                if (string.IsNullOrEmpty(file) || texByFile.ContainsKey(file)) continue;
                string src = Path.Combine(SimRoot, file);
                string dst = $"{k_GenDir}/Textures/{Path.GetFileName(file)}";
                File.Copy(src, Path.Combine(ProjectRoot, dst), true);
                AssetDatabase.ImportAsset(dst, ImportAssetOptions.ForceSynchronousImport);
                var ti = (TextureImporter)AssetImporter.GetAtPath(dst);
                ti.textureType = isNormal ? TextureImporterType.NormalMap : TextureImporterType.Default;
                ti.sRGBTexture = !isNormal;
                ti.mipmapEnabled = true;
                ti.anisoLevel = 4;
                ti.SaveAndReimport();
                texByFile[file] = AssetDatabase.LoadAssetAtPath<Texture2D>(dst);
            }

        var boardLabels = ImportBoardLabels(s);
        ImportHudLabels();

        // 재질
        var shader = Shader.Find("Standard");
        var mats = new Dictionary<string, Material>();
        foreach (var m in s.materials)
        {
            string path = $"{k_GenDir}/Materials/{m.name}.mat";
            var mat = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (mat == null) { mat = new Material(shader); AssetDatabase.CreateAsset(mat, path); }
            mat.shader = shader;
            mat.color = Col(m.color);
            mat.SetFloat("_Metallic", m.metallic);
            mat.SetFloat("_Glossiness", Mathf.Clamp01(m.smoothness));
            mat.SetTexture("_MainTex", !string.IsNullOrEmpty(m.albedo) ? texByFile[m.albedo] : null);
            if (!string.IsNullOrEmpty(m.normal)) { mat.SetTexture("_BumpMap", texByFile[m.normal]); mat.EnableKeyword("_NORMALMAP"); }
            else { mat.SetTexture("_BumpMap", null); mat.DisableKeyword("_NORMALMAP"); }
            EditorUtility.SetDirty(mat);
            mats[m.name] = mat;
        }

        // 메쉬
        var meshes = new Dictionary<string, Mesh>();
        foreach (var mj in s.meshes)
        {
            int n = mj.positions.Length / 3;
            var verts = new Vector3[n];
            for (int i = 0; i < n; i++) verts[i] = new Vector3(mj.positions[3 * i], mj.positions[3 * i + 1], mj.positions[3 * i + 2]);
            var mesh = new Mesh { name = mj.name, indexFormat = n > 65000 ? UnityEngine.Rendering.IndexFormat.UInt32 : UnityEngine.Rendering.IndexFormat.UInt16 };
            mesh.vertices = verts;
            if (mj.uvs != null && mj.uvs.Length == 2 * n)
            {
                var uv = new Vector2[n];
                for (int i = 0; i < n; i++) uv[i] = new Vector2(mj.uvs[2 * i], mj.uvs[2 * i + 1]);
                mesh.uv = uv;
            }
            mesh.triangles = Enumerable.Range(0, n).ToArray();
            if (mj.normals != null && mj.normals.Length == 3 * n)
            {
                var nor = new Vector3[n];
                for (int i = 0; i < n; i++) nor[i] = new Vector3(mj.normals[3 * i], mj.normals[3 * i + 1], mj.normals[3 * i + 2]);
                mesh.normals = nor;
            }
            else mesh.RecalculateNormals();
            if (mesh.uv != null && mesh.uv.Length == n) mesh.RecalculateTangents();
            mesh.RecalculateBounds();
            string path = $"{k_GenDir}/Meshes/{mj.name}.asset";
            if (AssetDatabase.LoadAssetAtPath<Mesh>(path) != null) AssetDatabase.DeleteAsset(path);
            AssetDatabase.CreateAsset(mesh, path);
            meshes[mj.name] = mesh;
        }

        // 씬 오브젝트 (지우고 다시)
        var old = GameObject.Find(k_Root);
        if (old != null) Object.DestroyImmediate(old);
        var root = new GameObject(k_Root).transform;
        var groups = new Dictionary<string, Transform>();
        Transform Group(string g)
        {
            if (groups.TryGetValue(g, out var t)) return t;
            t = new GameObject(g).transform; t.SetParent(root, false); groups[g] = t; return t;
        }
        int nObj = 0, nOnRobot = 0;
        foreach (var o in s.objects)
        {
            string g = o.mesh == "strawberry" ? "Strawberries" : o.mesh == "vine" ? "Vines" : "Props";
            var go = new GameObject(o.name);
            Transform parent = Group(g);
            if (!string.IsNullOrEmpty(o.parent))
            {
                // 로봇 링크 기준 형상(그리퍼의 D455 본체): 그 링크의 자식으로, 링크 로컬 좌표 그대로. 손목 카메라에서 뺄 레이어
                parent = FindDeep(robot.transform, o.parent);
                if (parent == null) { Debug.LogWarning($"[SimSetup] WARN 부모 링크 '{o.parent}' 없음 — {o.name} 건너뜀"); Object.DestroyImmediate(go); continue; }
                // 링크 아래 것은 위의 Environment 지우기에 걸리지 않는다 — 다시 빌드할 때 같은 이름의 예전 것을 지운다(안 지우면 겹쳐 쌓인다)
                foreach (var prev in parent.Cast<Transform>().Where(c => c.name == o.name).ToList()) Object.DestroyImmediate(prev.gameObject);
                go.layer = k_BodyLayer;
                nOnRobot++;
            }
            go.transform.SetParent(parent, false);
            go.transform.localPosition = V(o.pos); go.transform.localRotation = Q(o.rot); go.transform.localScale = V(o.scale);
            go.AddComponent<MeshFilter>().sharedMesh = meshes[o.mesh];
            var mr = go.AddComponent<MeshRenderer>(); mr.sharedMaterial = mats[o.material];
            nObj++;
        }
        var hl = BuildHighlights(s, Group("Highlights"), meshes);
        BuildBoardDisplay(root.gameObject, GameObject.Find($"{k_Root}/Props/board"), hl, boardLabels);
        BuildWristCamera(s, robot);
        var cube = Resources.GetBuiltinResource<Mesh>("Cube.fbx");
        foreach (var b in s.boxes)
        {
            var go = new GameObject(b.name);
            go.transform.SetParent(Group("Boxes"), false);
            go.transform.localPosition = V(b.pos); go.transform.localRotation = Q(b.rot); go.transform.localScale = V(b.scale);
            go.AddComponent<MeshFilter>().sharedMesh = cube;
            var mr = go.AddComponent<MeshRenderer>(); mr.sharedMaterial = mats[b.material];
            if (b.name == "floor") mr.receiveShadows = true;
        }
        Log($"Environment: 오브젝트 {nObj}(로봇 링크 아래 {nOnRobot}), 상자 {s.boxes.Length}, 재질 {mats.Count}, 메쉬 {meshes.Count}, 하이라이트 {hl.Count}, 보드 라벨 {boardLabels.Count} (Isaac 씬 정적 형상, 콜라이더 없음). 검사 {s.checks}");

        BuildLights(s);
        RobotMaterials();
        SetupCamera(s);
        DisablePlaceholderBerries();
    }

    static Transform FindDeep(Transform root, string name)
    {
        if (root.name == name) return root;
        foreach (Transform c in root) { var r = FindDeep(c, name); if (r != null) return r; }
        return null;
    }

    static Type RuntimeType(string name) =>
        AppDomain.CurrentDomain.GetAssemblies().SelectMany(a => { try { return a.GetTypes(); } catch { return new Type[0]; } }).FirstOrDefault(t => t.Name == name);

    static Texture2D ImportTexture(string src, string dst, bool gui, int maxSize)
    {
        File.Copy(src, Path.Combine(ProjectRoot, dst), true);
        AssetDatabase.ImportAsset(dst, ImportAssetOptions.ForceSynchronousImport);
        var ti = (TextureImporter)AssetImporter.GetAtPath(dst);
        ti.textureType = TextureImporterType.Default;
        ti.sRGBTexture = true;
        if (gui)
        {
            // HUD 라벨: 화면 픽셀에 그대로 — 밉맵·압축 없이, 가장자리 반복 없이
            ti.alphaIsTransparency = true; ti.mipmapEnabled = false; ti.npotScale = TextureImporterNPOTScale.None;
            ti.textureCompression = TextureImporterCompression.Uncompressed; ti.wrapMode = TextureWrapMode.Clamp; ti.filterMode = FilterMode.Bilinear;
        }
        else { ti.mipmapEnabled = true; ti.anisoLevel = 4; ti.maxTextureSize = maxSize; }
        ti.SaveAndReimport();
        return AssetDatabase.LoadAssetAtPath<Texture2D>(dst);
    }

    // 보드 방위 글자 텍스처 9장 (tree_model.board_label_key 의 키 → 격자 텍스처 사본)
    static List<(string key, Texture2D tex)> ImportBoardLabels(SceneJ s)
    {
        var list = new List<(string, Texture2D)>();
        if (s.board_label_textures == null) return list;
        EnsureFolder(k_GenDir + "/Textures/board_labels");
        foreach (var l in s.board_label_textures)
            list.Add((l.key, ImportTexture(Path.Combine(SimRoot, l.file), $"{k_GenDir}/Textures/board_labels/{Path.GetFileName(l.file)}", false, 2048)));
        return list;
    }

    // HUD 라벨 PNG + manifest.json (Isaac 표시 스크립트와 같은 파일, export_isaac_scene.py 가 assets_src/hud/labels 에 복사)
    static void ImportHudLabels()
    {
        string src = Path.Combine(SimRoot, "assets_src", "hud", "labels");
        if (!Directory.Exists(src)) { Debug.LogWarning("[SimSetup] WARN HUD 라벨 없음 — python3 -B scripts/export_isaac_scene.py"); return; }
        EnsureFolder(k_HudLabelDir);
        int n = 0;
        foreach (var f in Directory.GetFiles(src, "*.png")) { ImportTexture(f, $"{k_HudLabelDir}/{Path.GetFileName(f)}", true, 0); n++; }
        string man = Path.Combine(src, "manifest.json");
        if (File.Exists(man))
        {
            File.Copy(man, Path.Combine(ProjectRoot, k_HudLabelDir, "manifest.json"), true);
            AssetDatabase.ImportAsset($"{k_HudLabelDir}/manifest.json", ImportAssetOptions.ForceSynchronousImport);
        }
        Log($"HUD 라벨 PNG {n}장 + manifest → {k_HudLabelDir}");
    }

    // 보드 하이라이트 테두리 20장: Isaac Glow_Mat(청록 발광). 꺼 둔 채로 만들고 BoardDisplay 가 켠다
    static List<(string name, GameObject go)> BuildHighlights(SceneJ s, Transform group, Dictionary<string, Mesh> meshes)
    {
        var list = new List<(string, GameObject)>();
        if (s.highlights == null || s.highlights.Length == 0) return list;
        string path = $"{k_GenDir}/Materials/highlight_glow.mat";
        var mat = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (mat == null) { mat = new Material(Shader.Find("Standard")); AssetDatabase.CreateAsset(mat, path); }
        mat.shader = Shader.Find("Standard");
        Color glow = s.glow != null && s.glow.color != null ? Col(s.glow.color) : new Color(0.22f, 0.48f, 0.72f);
        // Isaac emissive 4 × 청록. 내장 렌더러(HDR 끔)에서는 발광색이 조명과 무관하게 더해진다 — 밝은 청록으로 보이게 1.6 배
        mat.color = glow;
        mat.SetFloat("_Metallic", 0f); mat.SetFloat("_Glossiness", 0.2f);
        mat.EnableKeyword("_EMISSION");
        mat.SetColor("_EmissionColor", glow * 1.6f);
        mat.globalIlluminationFlags = MaterialGlobalIlluminationFlags.None;
        EditorUtility.SetDirty(mat);
        foreach (var h in s.highlights)
        {
            if (!meshes.TryGetValue(h.mesh, out var mesh)) continue;
            var go = new GameObject(h.mesh);
            go.transform.SetParent(group, false);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            var mr = go.AddComponent<MeshRenderer>();
            mr.sharedMaterial = mat;
            mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off; mr.receiveShadows = false;
            go.SetActive(false);
            list.Add((h.name, go));
        }
        return list;
    }

    static void BuildBoardDisplay(GameObject root, GameObject board, List<(string name, GameObject go)> hl, List<(string key, Texture2D tex)> labels)
    {
        var type = RuntimeType("BoardDisplay");
        if (type == null) { Debug.LogWarning("[SimSetup] WARN BoardDisplay 타입 없음"); return; }
        var c = root.GetComponent(type);
        if (c == null) c = root.AddComponent(type);
        var so = new SerializedObject(c);
        so.FindProperty("board").objectReferenceValue = board != null ? board.GetComponent<Renderer>() : null;
        void Fill<T>(string prop, IList<T> items, Action<SerializedProperty, T> set)
        {
            var arr = so.FindProperty(prop); arr.ClearArray();
            for (int i = 0; i < items.Count; i++) { arr.InsertArrayElementAtIndex(i); set(arr.GetArrayElementAtIndex(i), items[i]); }
        }
        Fill("highlightNames", hl, (e, x) => e.stringValue = x.name);
        Fill("highlights", hl, (e, x) => e.objectReferenceValue = x.go);
        Fill("labelKeys", labels, (e, x) => e.stringValue = x.key);
        Fill("labelTextures", labels, (e, x) => e.objectReferenceValue = x.tex);
        so.ApplyModifiedPropertiesWithoutUndo();
        Log($"BoardDisplay: 보드 {(board != null ? board.name : "없음")}, 하이라이트 {hl.Count}, 방위 글자 텍스처 {labels.Count} [{string.Join(",", labels.Select(l => l.key))}]");
    }

    // 손목 D455 컬러 카메라: Isaac 카메라 프림의 rh_p12_rn_base 기준 자세·화각 그대로
    static void BuildWristCamera(SceneJ s, GameObject robot)
    {
        var w = s.wrist_camera;
        foreach (var old in robot.GetComponentsInChildren<Camera>(true)) Object.DestroyImmediate(old.gameObject);
        if (w == null || w.pos == null) { Debug.LogWarning("[SimSetup] WARN scene.json 에 wrist_camera 없음"); return; }
        var link = FindDeep(robot.transform, w.parent);
        if (link == null) { Debug.LogWarning($"[SimSetup] WARN 손목 카메라 부모 '{w.parent}' 없음"); return; }
        var go = new GameObject("Wrist Camera (D455 color)");
        go.transform.SetParent(link, false);
        go.transform.localPosition = V(w.pos);
        go.transform.localRotation = Quaternion.LookRotation(V(w.forward), V(w.up));
        var cam = go.AddComponent<Camera>();
        cam.fieldOfView = w.vfov_deg;
        cam.nearClipPlane = 0.01f; cam.farClipPlane = 30f;
        cam.clearFlags = CameraClearFlags.SolidColor; cam.backgroundColor = new Color(0.17f, 0.18f, 0.20f);
        cam.allowMSAA = true; cam.allowHDR = false; cam.depth = -2;
        var type = RuntimeType("WristCamera");
        if (type != null) go.AddComponent(type); else Debug.LogWarning("[SimSetup] WARN WristCamera 타입 없음");
        Log($"손목 카메라: {w.parent} 아래 {V(w.pos):F4}, 세로 {w.vfov_deg:F2}° × 가로 {w.hfov_deg:F2}°, {(w.res != null && w.res.Length == 2 ? w.res[0] + "×" + w.res[1] : "?")}");
    }

    // Isaac: 돔 라이트(전역 균일) + 위쪽 사각 조명 key_top. Built-in 렌더러는 실시간 면광원이 없어 방향광 + 환경광으로 옮긴다.
    static void BuildLights(SceneJ s)
    {
        foreach (var l in Object.FindObjectsOfType<Light>()) Object.DestroyImmediate(l.gameObject);
        var key = s.lights.FirstOrDefault(l => l.type == "RectLight");
        var kg = new GameObject("Key Light (Isaac key_top)");
        var k = kg.AddComponent<Light>();
        k.type = LightType.Directional;
        kg.transform.rotation = Quaternion.LookRotation(key != null ? V(key.forward) : Vector3.down, Vector3.forward);
        k.color = key != null ? Col(key.color) : Color.white;
        k.intensity = 1.0f;
        k.shadows = LightShadows.Soft; k.shadowStrength = 0.6f; k.shadowBias = 0.02f; k.shadowNormalBias = 0.2f;
        var fg = new GameObject("Fill Light");
        var f = fg.AddComponent<Light>();
        f.type = LightType.Directional;
        fg.transform.rotation = Quaternion.Euler(25f, 125f, 0f);
        f.intensity = 0.35f; f.shadows = LightShadows.None; f.color = new Color(0.95f, 0.97f, 1f);
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight;
        RenderSettings.ambientSkyColor = new Color(0.62f, 0.63f, 0.66f);
        RenderSettings.ambientEquatorColor = new Color(0.45f, 0.45f, 0.47f);
        RenderSettings.ambientGroundColor = new Color(0.25f, 0.25f, 0.26f);
        Log("조명: 방향광(key_top 방향) + 보조광 + 3색 환경광(돔 라이트 대신)");
    }

    // URDF-Importer 의 기본 재질은 금속성·광택 0.75 라 크롬처럼 보인다. 실제 로봇(도장된 흰색·회색)에 가깝게 무광으로.
    static void RobotMaterials()
    {
        var guids = AssetDatabase.FindAssets("t:Material", new[] { "Assets/URDF/e0509/Materials" });
        foreach (var g in guids)
        {
            var mat = AssetDatabase.LoadAssetAtPath<Material>(AssetDatabase.GUIDToAssetPath(g));
            mat.SetFloat("_Metallic", 0f);
            mat.SetFloat("_Glossiness", mat.name == "black" ? 0.25f : 0.38f);
            EditorUtility.SetDirty(mat);
        }
        Log($"로봇 재질 {guids.Length}개: 금속성 0, 광택 0.25–0.38 (도장면)");
    }

    // 카메라: 컨트롤러를 붙이고 Isaac 구도 프리셋 + 로봇 근접 구도를 채운다. 시작 구도 = 1번 Isaac 롱샷(Isaac 이 씬을 열 때 고정하는 구도,
    // 책상·보드·트레이 전체). 렌즈는 Isaac 과 같게 초점거리 + 센서 가로 20.955 mm, 가로 맞춤(SimCameraController.ApplyLens).
    // 순서: 1 isaac long · 2 isaac record close · 3 isaac side s · 4 isaac pair a2 · 5 isaac pair a3 · 6 robot close
    static void SetupCamera(SceneJ s)
    {
        var cam = Camera.main != null ? Camera.main : Object.FindObjectsOfType<Camera>().FirstOrDefault(c => c.GetComponent("WristCamera") == null);
        if (cam == null) { var go = new GameObject("Main Camera") { tag = "MainCamera" }; cam = go.AddComponent<Camera>(); go.AddComponent<AudioListener>(); }
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.backgroundColor = new Color(0.17f, 0.18f, 0.20f);
        cam.nearClipPlane = 0.02f; cam.farClipPlane = 30f;
        cam.allowMSAA = true; cam.allowHDR = false;
        cam.depth = 0;
        var type = RuntimeType("SimCameraController");
        if (type == null) { Debug.LogWarning("[SimSetup] WARN SimCameraController 타입 없음"); return; }
        var ctl = cam.GetComponent(type);                      // ?? 금지(Unity 가짜 null)
        if (ctl == null) ctl = cam.gameObject.AddComponent(type);
        var so = new SerializedObject(ctl);
        var list = so.FindProperty("presets");
        list.ClearArray();
        var all = new List<(string name, Vector3 pos, Vector3 fwd, Vector3 up, float vfov, float focal)>();
        foreach (var c in s.cameras)
        {
            string nm = c.name.Replace("isaac_", "isaac ").Replace("_", " ");
            all.Add((nm, V(c.pos), V(c.forward), V(c.up), c.vfov_deg, c.focal_mm));
        }
        // 로봇·보드 근접 측면(Isaac 프리셋에 없는 조작 시작점). 16:9 에서 세로 40° 이던 화면과 같은 가로 화각
        var closePos = new Vector3(0.55f, 1.05f, 0.95f);
        var target = new Vector3(-0.45f, 0.75f, 0.05f);
        all.Add(("robot close", closePos, (target - closePos).normalized, Vector3.up, 40f, k_RobotCloseFocal));
        for (int i = 0; i < all.Count; i++)
        {
            list.InsertArrayElementAtIndex(i);
            var e = list.GetArrayElementAtIndex(i);
            e.FindPropertyRelative("name").stringValue = all[i].name;
            e.FindPropertyRelative("position").vector3Value = all[i].pos;
            e.FindPropertyRelative("forward").vector3Value = all[i].fwd;
            e.FindPropertyRelative("up").vector3Value = all[i].up;
            e.FindPropertyRelative("verticalFov").floatValue = all[i].vfov;
            e.FindPropertyRelative("focalLength").floatValue = all[i].focal;
        }
        int start = Mathf.Max(0, all.FindIndex(a => a.name == "isaac long"));
        so.FindProperty("startPreset").intValue = start;
        so.FindProperty("sensorWidthMm").floatValue = 20.955f;
        so.ApplyModifiedPropertiesWithoutUndo();
        // 에디터에서 보이는 처음 화면도 시작 구도로
        var sp = all[start];
        cam.transform.position = sp.pos; cam.transform.rotation = Quaternion.LookRotation(sp.fwd, sp.up);
        SetLens(cam, sp.focal, sp.vfov);
        Log($"카메라: 프리셋 {all.Count}개 [{string.Join(", ", all.Select((a, i) => $"{i + 1} {a.name} {a.focal:F1}mm"))}], 시작 '{sp.name}', 가로 맞춤 센서 20.955 mm");
    }

    // SimCameraController.ApplyLens 와 같은 설정(편집 모드·렌더 미리보기용)
    static void SetLens(Camera cam, float focal, float vfov)
    {
        if (focal > 0f)
        {
            cam.usePhysicalProperties = true;
            cam.sensorSize = new Vector2(20.955f, 20.955f * 9f / 16f);
            cam.gateFit = Camera.GateFitMode.Horizontal;
            cam.lensShift = Vector2.zero;
            cam.focalLength = focal;
        }
        else { cam.usePhysicalProperties = false; cam.fieldOfView = vfov; }
    }

    // Isaac 딸기 메쉬가 생겼으니 StrawberryPublisher 의 자리표시 구는 만들지 않는다
    static void DisablePlaceholderBerries()
    {
        var type = AppDomain.CurrentDomain.GetAssemblies().SelectMany(a => { try { return a.GetTypes(); } catch { return new Type[0]; } }).FirstOrDefault(t => t.Name == "StrawberryPublisher");
        if (type == null) return;
        foreach (Component c in Object.FindObjectsOfType(type))
        {
            var so = new SerializedObject(c);
            var p = so.FindProperty("buildPlaceholderSpheres");
            if (p != null) { p.boolValue = false; so.ApplyModifiedPropertiesWithoutUndo(); }
            var old = c.transform.Find("Strawberries");
            if (old != null) Object.DestroyImmediate(old.gameObject);
        }
    }

    // ── 화질: 모든 품질 단계에 MSAA 4x, 부드러운 그림자, 높은 그림자 해상도 ──
    public static void ConfigureQuality()
    {
        var qs = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/QualitySettings.asset").FirstOrDefault();
        if (qs == null) { Debug.LogWarning("[SimSetup] WARN QualitySettings.asset 없음"); return; }
        var so = new SerializedObject(qs);
        var arr = so.FindProperty("m_QualitySettings");
        for (int i = 0; i < arr.arraySize; i++)
        {
            var e = arr.GetArrayElementAtIndex(i);
            e.FindPropertyRelative("antiAliasing").intValue = 4;
            e.FindPropertyRelative("shadows").intValue = 2;            // 0 끔, 1 hard, 2 soft
            e.FindPropertyRelative("shadowResolution").intValue = 3;   // VeryHigh
            e.FindPropertyRelative("shadowDistance").floatValue = 8f;
            e.FindPropertyRelative("anisotropicTextures").intValue = 2;
        }
        so.ApplyModifiedPropertiesWithoutUndo();
        Log($"화질: 품질 단계 {arr.arraySize}개 모두 MSAA 4x, soft 그림자 VeryHigh, 그림자 거리 8 m");
    }

    // ══════════════════════════════════════════════════════════════════════
    // 렌더 미리보기: 로봇을 overview 자세로 놓고(편집 모드에서 링크 회전만, 씬 저장 안 함) 프리셋마다 PNG.
    // 덧붙여: 롱샷 4:3(가로 맞춤 확인 — 16:9 와 좌우 범위가 같아야 한다) · 계란판 근접 3방향(양면 확인) · 그리퍼 D455 근접 ·
    //         손목 카메라 화면 · 하이라이트·방위 글자 시연(ne 분면을 켠 롱샷)
    [MenuItem("Sim/Render Preview")]
    public static void RenderPreview()
    {
        try
        {
            EditorSceneManager.OpenScene("Assets/Scenes/E0509Sim.unity", OpenSceneMode.Single);
            var robot = Object.FindObjectOfType<UrdfRobot>();
            ApplyPreviewPose(robot.gameObject, new Dictionary<string, float>
            {
                ["joint_1"] = 87.98f, ["joint_2"] = -94.92f, ["joint_3"] = 129.89f, ["joint_4"] = 175.94f, ["joint_5"] = -31.34f, ["joint_6"] = 93.42f,
                ["rh_p12_rn"] = 53.05f, ["rh_r2"] = 53.05f, ["rh_l1"] = 53.05f, ["rh_l2"] = 53.05f,
            });
            var cam = Camera.main;
            var ctl = cam.GetComponent(RuntimeType("SimCameraController"));
            var so = new SerializedObject(ctl);
            var list = so.FindProperty("presets");
            string outDir = Path.Combine(SimRoot, "logs", "render");
            Directory.CreateDirectory(outDir);
            void Shot(Camera c, int w, int h, string name)
            {
                var rt = new RenderTexture(w, h, 24) { antiAliasing = 4 };
                var prev = c.targetTexture;
                c.targetTexture = rt;
                c.ResetAspect();                 // 화면비 = 이 렌더 크기 (가로 맞춤이면 세로 화각이 따라 바뀐다)
                c.Render();
                RenderTexture.active = rt;
                var tex = new Texture2D(w, h, TextureFormat.RGB24, false);
                tex.ReadPixels(new Rect(0, 0, w, h), 0, 0); tex.Apply();
                string file = Path.Combine(outDir, name + ".png");
                File.WriteAllBytes(file, tex.EncodeToPNG());
                RenderTexture.active = null; c.targetTexture = prev;
                Object.DestroyImmediate(tex); Object.DestroyImmediate(rt);
                Log($"렌더 → {file} ({w}×{h}, 세로 화각 {c.fieldOfView:F2}°)");
            }
            void Preset(int i)
            {
                var e = list.GetArrayElementAtIndex(i);
                cam.transform.position = e.FindPropertyRelative("position").vector3Value;
                cam.transform.rotation = Quaternion.LookRotation(e.FindPropertyRelative("forward").vector3Value, e.FindPropertyRelative("up").vector3Value);
                SetLens(cam, e.FindPropertyRelative("focalLength").floatValue, e.FindPropertyRelative("verticalFov").floatValue);
            }
            void Look(Vector3 pos, Vector3 at, float vfov) { cam.transform.position = pos; cam.transform.rotation = Quaternion.LookRotation(at - pos, Vector3.up); SetLens(cam, 0f, vfov); }
            for (int i = 0; i < list.arraySize; i++)
            {
                Preset(i);
                Shot(cam, 1600, 900, $"{i + 1}_{list.GetArrayElementAtIndex(i).FindPropertyRelative("name").stringValue.Replace(' ', '_')}");
            }
            Preset(0); Shot(cam, 1200, 900, "1_isaac_long_4x3");
            Preset(0); Shot(cam, 1000, 1000, "1_isaac_long_1x1");

            // 계란판: 위 · 앞(로봇 반대쪽 +z 에서) · 롱샷 방향 근접
            var carton = GameObject.Find($"{k_Root}/Props/egg_carton");
            if (carton != null)
            {
                var c = carton.GetComponent<Renderer>().bounds.center;
                Look(c + new Vector3(0.001f, 0.45f, -0.02f), c, 40f); Shot(cam, 1200, 900, "carton_top");
                Look(c + new Vector3(0f, 0.12f, 0.42f), c, 40f); Shot(cam, 1200, 900, "carton_front");
                Look(c + new Vector3(0.38f, 0.22f, -0.30f), c, 40f); Shot(cam, 1200, 900, "carton_long_dir");
            }
            // 그리퍼 D455 근접
            var flange = FindDeep(robot.transform, "rh_p12_rn_base");
            if (flange != null)
            {
                var g = flange.position;
                Look(g + new Vector3(0.28f, 0.10f, -0.22f), g + flange.up * 0.08f, 35f); Shot(cam, 1200, 900, "gripper_d455");
            }
            // 손목 카메라 화면 (본체 레이어 빼고)
            var wc = robot.GetComponentsInChildren<Camera>(true).FirstOrDefault();
            if (wc != null) { int mask = wc.cullingMask; wc.cullingMask &= ~(1 << k_BodyLayer); Shot(wc, 640, 480, "wrist_camera"); wc.cullingMask = mask; }
            // 하이라이트·방위 글자 시연: ne 분면 테두리 + q1_ne 글자 (편집 모드에서만, 저장 안 함)
            var bd = Object.FindObjectOfType(RuntimeType("BoardDisplay")) as MonoBehaviour;
            if (bd != null)
            {
                var m = bd.GetType().GetMethod("Apply");
                m.Invoke(bd, new object[] { new[] { "ne" }, "q1_ne" });
                Preset(0); Shot(cam, 1600, 900, "board_hl_q1_ne");
                m.Invoke(bd, new object[] { new[] { "nw", "ne", "sw", "se" }, "home" });
                Preset(0); Shot(cam, 1600, 900, "board_hl_home");
                m.Invoke(bd, new object[] { new string[0], "" });
            }
            Log($"RenderPreview OK (그래픽 장치 {SystemInfo.graphicsDeviceType})");
        }
        catch (Exception e)
        {
            Debug.LogError("[SimSetup] FAIL RenderPreview: " + e);
            if (Application.isBatchMode) EditorApplication.Exit(1);
        }
    }

    // 편집 모드에서 관절각대로 링크를 돌린다. 축은 ArticulationBody anchor X, 부호는 overview TCP 위치로 정한다.
    static void ApplyPreviewPose(GameObject robot, Dictionary<string, float> deg)
    {
        var joints = robot.GetComponentsInChildren<UrdfJoint>(true).Where(j => j.JointType == UrdfJoint.JointTypes.Revolute).ToList();
        var rest = joints.ToDictionary(j => j, j => j.transform.localRotation);
        var tcp = GameObject.Find("tcp")?.transform;
        Vector3 expect = new Vector3(-0.2151f, 0.6426f, 0.0226f);   // ROS (0.0226, 0.2151, 0.6426) → Unity
        float best = float.MaxValue; float bestSign = 1;
        foreach (float sign in new[] { 1f, -1f })
        {
            foreach (var j in joints)
            {
                var ab = j.GetComponent<ArticulationBody>();
                if (!deg.TryGetValue(j.jointName, out var a)) continue;
                Vector3 axis = ab.anchorRotation * Vector3.right;
                j.transform.localRotation = rest[j] * Quaternion.AngleAxis(sign * a, axis);
            }
            float err = tcp != null ? Vector3.Distance(tcp.position, expect) : 0f;
            Log($"미리보기 자세 부호 {sign:+0;-0}: TCP {(tcp != null ? tcp.position.ToString("F4") : "?")} 기대 {expect:F4} 오차 {err * 1000:F1} mm");
            if (err < best) { best = err; bestSign = sign; }
        }
        foreach (var j in joints)
        {
            var ab = j.GetComponent<ArticulationBody>();
            if (!deg.TryGetValue(j.jointName, out var a)) continue;
            j.transform.localRotation = rest[j] * Quaternion.AngleAxis(bestSign * a, ab.anchorRotation * Vector3.right);
        }
        Log($"미리보기 자세: overview, 부호 {bestSign:+0;-0}, TCP 오차 {best * 1000:F1} mm");
    }
}
