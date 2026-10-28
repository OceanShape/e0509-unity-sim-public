// 단계 5 — 관절 상태 미러/명령 적용 (CONTRACT §9-2 D-4).
//
// 받는 토픽(기본 /dsr01/joint_states, sensor_msgs/JointState)의 관절을 **이름**으로 ArticulationBody 에 매핑한다.
// 순서에 의존하지 않는다(CONTRACT §2-2: Isaac 상태 순서와 가상 제어기 명령 순서가 다르다).
//
//  Mirror  (M1) : 받는 값이 이미 측정값 → xDrive.target(deg) 과 jointPosition(rad) 둘 다 대입. 팔이 그대로 따라간다.
//  Command (M2) : xDrive.target 만 준다. jointPosition 은 시작 자세 초기화 때만. 회신은 실제로 움직인 jointPosition 이다.
//                 대입하면 측정값이 언제나 명령과 같아져 도착 판정이 항상 즉시 통과한다(원본보다 관대 → 금지).
//
// 콜백은 최신 메시지만 보관하고 적용은 FixedUpdate 에서 한다. ROS-TCP-Connector 는 콜백을 메인 스레드(ROSConnection.Update)
// 에서 부르므로 잠금은 필요 없다.
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.Sensor;

public class JointStateMirror : MonoBehaviour
{
    public enum ApplyMode { Mirror, Command }

    [Header("ROS")]
    [Tooltip("Mirror(M1): mirrorTopic 의 측정값을 그대로 재생. Command(M2): commandTopic 의 명령을 드라이브 목표로만 적용")]
    public ApplyMode mode = ApplyMode.Mirror;
    [Tooltip("M1 구독 토픽 — Isaac 상태 토픽 (CONTRACT §3-1)")]
    public string mirrorTopic = "/dsr01/joint_states";
    [Tooltip("M2 구독 토픽 — 가상 제어기 명령 (CONTRACT §3-2)")]
    public string commandTopic = "/joint_command";
    /// 지금 모드의 구독 토픽
    public string topic => mode == ApplyMode.Command ? commandTopic : mirrorTopic;

    /// 테스트·배치 실행용 모드 강제. 씬 로드 전에 설정하면 Awake 에서 덮어쓴다.
    /// 명령줄 `-simMode command|mirror` 또는 환경 변수 SIM_MODE 로도 줄 수 있다.
    public static ApplyMode? ForcedMode;
    /// 테스트용: true 면 ROS 에 연결·구독하지 않는다(엔드포인트 없이 물리만 시험).
    public static bool SkipRos;
    /// 테스트용 드라이브 조정(씬 값 덮어쓰기). null 이면 씬(SimSetup) 값 그대로.
    public static float? OverrideStiffness, OverrideDamping, OverrideForceLimit;
    public static bool? OverrideVelocityLimits;

    [Header("Joint velocity limit (Isaac 과 같게 PhysX 로 강제)")]
    [Tooltip("CONTRACT §2-4 속도 한계를 ArticulationBody.maxJointVelocity 에 건다. Isaac 은 PhysX maxJointVelocity 로 같은 값을 강제한다")]
    public bool enforceVelocityLimits = true;

    [Header("Robot")]
    [Tooltip("비우면 씬의 UrdfRobot 을 찾는다")]
    public UrdfRobot robot;

    [Header("Start pose (deg) — CONTRACT §3-6 overview, 그리퍼 0.926 rad")]
    public bool applyStartPose = true;
    public string[] startPoseJoints = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "rh_p12_rn", "rh_r2", "rh_l1", "rh_l2" };
    public float[] startPoseDeg = { 87.98f, -94.92f, 129.89f, 175.94f, -31.34f, 93.42f, 53.05f, 53.05f, 53.05f, 53.05f };

    // ── 바깥(SimHud·모니터·상태 발행기)에 노출 ──
    public float ReceiveHz { get; private set; }
    public double LastReceiveRealtime { get; private set; } = double.NegativeInfinity; // Time.realtimeSinceStartupAsDouble
    public long MessageCount { get; private set; }
    public JointStateMsg LastMessage { get; private set; }
    public ROSConnection Ros { get; private set; }
    public bool Connected => Ros != null && Ros.HasConnectionThread && !Ros.HasConnectionError;
    public IReadOnlyDictionary<string, ArticulationBody> Joints => m_Joints;
    /// 메시지에 있었지만 매핑에 없는 이름(이름당 1회 경고)
    public IReadOnlyCollection<string> UnknownNames => m_Unknown;

    readonly Dictionary<string, ArticulationBody> m_Joints = new Dictionary<string, ArticulationBody>();
    readonly HashSet<string> m_Unknown = new HashSet<string>();
    readonly Queue<double> m_ArrivalTimes = new Queue<double>();   // 최근 1초 수신 시각 (Hz 계산)
    JointStateMsg m_Pending;
    bool m_HasPending;

    void Awake()
    {
        if (robot == null) robot = FindObjectOfType<UrdfRobot>();
        if (robot == null) { Debug.LogError("[JointStateMirror] 씬에 UrdfRobot 이 없다"); enabled = false; return; }

        // UrdfJoint 는 자식 링크 GameObject 에 ArticulationBody 와 함께 있다. 고정 관절은 건너뛴다.
        foreach (var j in robot.GetComponentsInChildren<UrdfJoint>(true))
        {
            if (j.JointType == UrdfJoint.JointTypes.Fixed) continue;
            var ab = j.GetComponent<ArticulationBody>();
            if (ab == null || string.IsNullOrEmpty(j.jointName)) continue;
            if (m_Joints.ContainsKey(j.jointName)) { Debug.LogWarning($"[JointStateMirror] 관절 이름 중복: {j.jointName}"); continue; }
            m_Joints[j.jointName] = ab;
        }
        Debug.Log($"[JointStateMirror] 관절 {m_Joints.Count}개 매핑: {string.Join(", ", m_Joints.Keys)}");

        ApplyModeOverride();
        if (OverrideVelocityLimits.HasValue) enforceVelocityLimits = OverrideVelocityLimits.Value;
        if (enforceVelocityLimits) ApplyVelocityLimits();
        ApplySolverIterations();
        // 플랜트는 항상 시뮬레이션한다: PhysX 가 정지한 아티큘레이션을 재우면 드라이브 목표를 바꿔도 깨어나지 않아
        // 그리퍼 닫기 명령이 첫 스텝 뒤 멈췄다(단계 7 CommandModeTest). Isaac 은 명령을 적용할 때마다 깨운다.
        foreach (var ab in robot.GetComponentsInChildren<ArticulationBody>(true)) ab.sleepThreshold = 0f;
        if (OverrideStiffness.HasValue || OverrideDamping.HasValue || OverrideForceLimit.HasValue)
        {
            foreach (var ab in m_Joints.Values)
            {
                var d = ab.xDrive;
                if (OverrideStiffness.HasValue) d.stiffness = OverrideStiffness.Value;
                if (OverrideDamping.HasValue) d.damping = OverrideDamping.Value;
                if (OverrideForceLimit.HasValue) d.forceLimit = OverrideForceLimit.Value;
                ab.xDrive = d;
            }
            Debug.Log($"[JointStateMirror] 드라이브 덮어쓰기 k={OverrideStiffness} c={OverrideDamping} f={OverrideForceLimit}");
        }
    }

    void ApplyModeOverride()
    {
        ApplyMode? m = ForcedMode;
        string src = "ForcedMode";
        if (m == null)
        {
            string arg = null;
            var args = Environment.GetCommandLineArgs();
            for (int i = 0; i + 1 < args.Length; i++) if (args[i] == "-simMode") arg = args[i + 1];
            if (arg == null) { arg = Environment.GetEnvironmentVariable("SIM_MODE"); src = "SIM_MODE"; } else src = "-simMode";
            if (!string.IsNullOrEmpty(arg))
                m = arg.Trim().ToLowerInvariant().StartsWith("c") ? ApplyMode.Command : ApplyMode.Mirror;
        }
        if (m != null && m.Value != mode)
        {
            Debug.Log($"[JointStateMirror] 모드 {mode} → {m.Value} ({src})");
            mode = m.Value;
        }
    }

    // CONTRACT §2-4: J1–J3 180 °/s, J4–J6 225 °/s, 그리퍼 372.4 °/s (xacro·URDF·USD·cuRobo 공통).
    // URDF-Importer 는 이 값을 maxAngularVelocity(몸체 각속도, 직렬화 안 됨)에 넣어 실제로는 아무것도 막지 않는다(단계 4 기록).
    // 여기서 Play 때마다 maxJointVelocity 에 다시 건다. 단위는 PhysX 축소 좌표 그대로(회전 관절 rad/s) — 단계 7 시험으로 확인.
    static readonly Dictionary<string, float> k_VelLimitDegPerSec = new Dictionary<string, float>
    {
        ["joint_1"] = 180f, ["joint_2"] = 180f, ["joint_3"] = 180f,
        ["joint_4"] = 225f, ["joint_5"] = 225f, ["joint_6"] = 225f,
        ["rh_p12_rn"] = 372.4f, ["rh_r2"] = 372.4f, ["rh_l1"] = 372.4f, ["rh_l2"] = 372.4f,
    };
    public static float VelocityLimitDegPerSec(string joint) => k_VelLimitDegPerSec.TryGetValue(joint, out var v) ? v : float.PositiveInfinity;

    // Isaac: robot_physics.usd root_joint physxArticulation:solverPositionIterationCount 32, solverVelocityIterationCount 1.
    // ArticulationBody.solverIterations 는 씬에 저장되지 않아(단계 7 확인) Play 때마다 루트에 건다. 테스트는 OverrideSolverIterations 로 바꾼다.
    public static int? OverrideSolverIterations;
    void ApplySolverIterations()
    {
        int it = OverrideSolverIterations ?? 32;
        foreach (var ab in robot.GetComponentsInChildren<ArticulationBody>(true))
        {
            if (!ab.isRoot) continue;
            ab.solverIterations = it;
            ab.solverVelocityIterations = 1;
            Debug.Log($"[JointStateMirror] 루트 {ab.name} 관절 풀이 반복 {ab.solverIterations}/{ab.solverVelocityIterations} (Isaac 32/1), 풀이기 {Physics.defaultSolverIterations}/{Physics.defaultSolverVelocityIterations} 기본");
        }
    }

    void ApplyVelocityLimits()
    {
        foreach (var kv in m_Joints)
            if (k_VelLimitDegPerSec.TryGetValue(kv.Key, out var deg))
                kv.Value.maxJointVelocity = deg * Mathf.Deg2Rad;
        Debug.Log("[JointStateMirror] maxJointVelocity 적용 (rad/s): " + string.Join(", ", m_Joints.Select(kv => $"{kv.Key}={kv.Value.maxJointVelocity:F3}")));
    }

    void Start()
    {
        if (applyStartPose) ApplyStartPose();
        if (SkipRos) { Debug.Log($"[JointStateMirror] SkipRos — ROS 구독 안 함 (mode={mode})"); return; }
        Ros = ROSConnection.GetOrCreateInstance();
        Ros.Subscribe<JointStateMsg>(topic, OnJointState);
        Debug.Log($"[JointStateMirror] 구독 {topic}  mode={mode}");
    }

    // 시작 자세: 두 모드 모두 jointPosition 과 target 을 함께 맞춘다(Command 모드에서 jointPosition 을 쓰는 유일한 곳).
    void ApplyStartPose()
    {
        int n = Mathf.Min(startPoseJoints.Length, startPoseDeg.Length);
        for (int i = 0; i < n; i++)
        {
            if (!m_Joints.TryGetValue(startPoseJoints[i], out var ab)) continue;
            SetJoint(ab, startPoseDeg[i] * Mathf.Deg2Rad, writePosition: true);
        }
    }

    /// 테스트용: ROS 콜백과 같은 경로로 메시지를 넣는다(최신 1개만, 다음 FixedUpdate 에 적용).
    public void InjectMessage(JointStateMsg msg) => OnJointState(msg);

    void OnJointState(JointStateMsg msg)
    {
        m_Pending = msg;
        m_HasPending = true;
        LastMessage = msg;
        MessageCount++;
        double now = Time.realtimeSinceStartupAsDouble;
        LastReceiveRealtime = now;
        m_ArrivalTimes.Enqueue(now);
    }

    void Update()
    {
        double now = Time.realtimeSinceStartupAsDouble;
        while (m_ArrivalTimes.Count > 0 && now - m_ArrivalTimes.Peek() > 1.0) m_ArrivalTimes.Dequeue();
        ReceiveHz = m_ArrivalTimes.Count;   // 최근 1초 수신 수
        if (now - LastReceiveRealtime > 1.0) ReceiveHz = 0f;
    }

    void FixedUpdate()
    {
        if (!m_HasPending) return;
        m_HasPending = false;
        Apply(m_Pending);
    }

    ArticulationBody m_Root;

    void Apply(JointStateMsg msg)
    {
        if (m_Root == null) m_Root = robot.GetComponentsInChildren<ArticulationBody>(true).FirstOrDefault(b => b.isRoot);
        if (m_Root != null) m_Root.WakeUp();
        int n = Mathf.Min(msg.name.Length, msg.position.Length);
        if (msg.name.Length != msg.position.Length)
            Debug.LogWarning($"[JointStateMirror] name {msg.name.Length}개 vs position {msg.position.Length}개 — 앞 {n}개만 적용");
        for (int i = 0; i < n; i++)
        {
            if (!m_Joints.TryGetValue(msg.name[i], out var ab))
            {
                if (m_Unknown.Add(msg.name[i]))
                    Debug.LogWarning($"[JointStateMirror] 매핑에 없는 관절 이름 '{msg.name[i]}' — 무시 (이 경고는 이름당 1회)");
                continue;
            }
            SetJoint(ab, (float)msg.position[i], writePosition: mode == ApplyMode.Mirror);
        }
    }

    // 임포터 관례(UrdfJointRevolute.OnUpdateJointState/GetPosition): target 은 deg, jointPosition 은 rad, 부호 변환 없음
    // (anchor 가 -axis 로 잡혀 있어 좌우손 좌표계 뒤집힘을 흡수한다). 프리즘 관절은 둘 다 m.
    static void SetJoint(ArticulationBody ab, float q, bool writePosition)
    {
        ArticulationDrive d = ab.xDrive;
        d.target = ab.jointType == ArticulationJointType.PrismaticJoint ? q : q * Mathf.Rad2Deg;
        ab.xDrive = d;
        if (writePosition)
        {
            // 위치를 대입해도 PhysX 는 남아 있는 관절 속도로 한 스텝 더 적분해 측정값이 0.5° 안팎 흐른다(M1 스모크 테스트 20:22).
            // 미러는 받은 값이 곧 측정값이어야 하므로 속도도 0 으로 둔다. 속도 감시는 JointLimitMonitor 가 위치 차분으로 한다.
            ab.jointPosition = new ArticulationReducedSpace(q);
            ab.jointVelocity = new ArticulationReducedSpace(0f);
        }
    }

    /// 측정 관절 위치(rad). 회신(M2 상태 발행)과 HUD 가 쓴다.
    public bool TryGetMeasuredRad(string jointName, out float rad)
    {
        if (m_Joints.TryGetValue(jointName, out var ab)) { rad = ab.jointPosition[0]; return true; }
        rad = 0f; return false;
    }
}
