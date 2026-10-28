// 단계 5 — 관절 한계 모니터 (CONTRACT §2-4·§2-5, D-3).
//
// Unity 시뮬레이터가 단순 미러가 아니라 모니터 역할도 한다는 근거. 단계 3 URDF 의 한계(= Isaac 합성값)를 기준으로:
//  - 위치 위반:  **여유 없이** |q| > 한계 (D-3 확정). 진입→이탈을 한 건으로 세고 최댓값을 기록한다.
//  - 한계 근처:  한계 안쪽 0.5° 이내 구간을 위반과 따로 진입·이탈 단위로 기록한다(시작 자세 J2 −94.92° 는 한계까지 0.08°).
//  - 운용 한계(J1 ±225, J5 ±130, J6 ±225): 위반으로 세지 않고 kind=operational 행으로만 남긴다(counted=0).
//  - 그리퍼: 0–1.1 rad 이 하드 한계. 1.0 rad(57.30°) 초과는 kind=gripper_over_1rad 정보 행(counted=0).
//  - 속도: CONTRACT §2-4 속도 한계. jointPosition 의 유한차(0.1 s 창)로 잰다. Mirror 모드에서 jointPosition 은 매 스텝
//    대입되므로 PhysX 의 jointVelocity 가 아니라 위치 차분이 "실측"이다. Command 모드에서도 같은 식이 성립한다.
//
// 기록: <SIM_ROOT>/logs/limit_violations.csv, <SIM_ROOT>/logs/limit_near.csv (SIM_ROOT = UnityProject 의 상위 폴더).
// Play 마다 덮어쓴다. 0건이어도 헤더는 쓴다. 근거로 남길 때는 logs/m1/ 등으로 복사한다.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using UnityEngine;
using Unity.Robotics.UrdfImporter;

public class JointLimitMonitor : MonoBehaviour
{
    [Serializable]
    public class JointLimit
    {
        public string joint;
        public float lowerDeg, upperDeg;     // 하드 한계 (Isaac 합성값 = 단계 3 URDF)
        public float velocityDegPerSec;      // 속도 한계
        public float opLowerDeg = float.NaN, opUpperDeg = float.NaN;   // 운용 한계 (경고만). NaN = 없음
        public float infoUpperDeg = float.NaN;                         // 정보 행 기준 (그리퍼 1.0 rad)
    }

    public UrdfRobot robot;
    public JointStateMirror mirror;                 // ros stamp 를 CSV 에 함께 적기 위해(선택)
    public float nearMarginDeg = 0.5f;              // 한계 근처 판정 폭
    public int velocityWindowSteps = 6;             // 60 Hz 에서 0.1 s
    public bool checkAgainstDriveLimits = true;     // 시작 시 ArticulationBody xDrive 한계와 표를 비교해 어긋나면 경고
    public string violationsCsvName = "limit_violations.csv";
    public string nearCsvName = "limit_near.csv";

    // CONTRACT §2-4·§2-5: Isaac 합성값. 그리퍼 1.1 rad = 63.0254°, 1.0 rad = 57.2958°. 속도 J1–3 180, J4–6 225, 그리퍼 372.4 °/s.
    public List<JointLimit> limits = new List<JointLimit>
    {
        new JointLimit{ joint="joint_1", lowerDeg=-360f, upperDeg=360f, velocityDegPerSec=180f, opLowerDeg=-225f, opUpperDeg=225f },
        new JointLimit{ joint="joint_2", lowerDeg= -95f, upperDeg= 95f, velocityDegPerSec=180f },
        new JointLimit{ joint="joint_3", lowerDeg=-135f, upperDeg=135f, velocityDegPerSec=180f },
        new JointLimit{ joint="joint_4", lowerDeg=-360f, upperDeg=360f, velocityDegPerSec=225f },
        new JointLimit{ joint="joint_5", lowerDeg=-135f, upperDeg=135f, velocityDegPerSec=225f, opLowerDeg=-130f, opUpperDeg=130f },
        new JointLimit{ joint="joint_6", lowerDeg=-360f, upperDeg=360f, velocityDegPerSec=225f, opLowerDeg=-225f, opUpperDeg=225f },
        new JointLimit{ joint="rh_p12_rn", lowerDeg=0f, upperDeg=63.0254f, velocityDegPerSec=372.4f, infoUpperDeg=57.2958f },
        new JointLimit{ joint="rh_r2",     lowerDeg=0f, upperDeg=63.0254f, velocityDegPerSec=372.4f, infoUpperDeg=57.2958f },
        new JointLimit{ joint="rh_l1",     lowerDeg=0f, upperDeg=63.0254f, velocityDegPerSec=372.4f, infoUpperDeg=57.2958f },
        new JointLimit{ joint="rh_l2",     lowerDeg=0f, upperDeg=63.0254f, velocityDegPerSec=372.4f, infoUpperDeg=57.2958f },
    };

    // ── 바깥에 노출 ──
    public int ViolationCount { get; private set; }        // counted=1 인 에피소드 수 (position + velocity)
    public int NearCount { get; private set; }
    public int OperationalWarnCount { get; private set; }
    public bool WrapDetected { get; private set; }         // J1/J4/J6 jointPosition 이 ±180° 에서 감싸는지 (CONTRACT §9-3)
    public string[] ActiveViolationJoints => m_Episodes.Where(e => e.active && e.counted).Select(e => e.joint).Distinct().ToArray();
    public string ViolationsCsvPath { get; private set; }
    public string NearCsvPath { get; private set; }

    class Tracked
    {
        public JointLimit lim; public ArticulationBody ab;
        public float[] history; public int histCount; public int histHead;   // 유한차 속도용 위치 링버퍼 (deg, 펼친 값)
        public float lastRawDeg; public float unwrapOffsetDeg; public bool hasLast;
    }
    class Episode
    {
        public string kind; public string joint; public bool counted; public bool active;
        public double enterT, exitT; public float peak; public float limit; public int samples; public string rosStampEnter;
        public StreamWriter writer;
    }

    readonly List<Tracked> m_Tracked = new List<Tracked>();
    readonly List<Episode> m_Episodes = new List<Episode>();
    StreamWriter m_Violations, m_Near;
    string m_Session;

    void Start()
    {
        if (robot == null) robot = FindObjectOfType<UrdfRobot>();
        if (mirror == null) mirror = GetComponent<JointStateMirror>();          // ?? 금지: UnityEngine.Object 의 가짜 null
        if (mirror == null) mirror = FindObjectOfType<JointStateMirror>();
        if (robot == null) { Debug.LogError("[JointLimitMonitor] 씬에 UrdfRobot 이 없다"); enabled = false; return; }

        var byName = new Dictionary<string, ArticulationBody>();
        foreach (var j in robot.GetComponentsInChildren<UrdfJoint>(true))
            if (j.JointType != UrdfJoint.JointTypes.Fixed && j.GetComponent<ArticulationBody>() != null) byName[j.jointName] = j.GetComponent<ArticulationBody>();

        foreach (var lim in limits)
        {
            if (!byName.TryGetValue(lim.joint, out var ab)) { Debug.LogWarning($"[JointLimitMonitor] 한계 표의 관절 '{lim.joint}' 이 로봇에 없다"); continue; }
            var t = new Tracked { lim = lim, ab = ab, history = new float[Mathf.Max(2, velocityWindowSteps + 1)] };
            m_Tracked.Add(t);
            if (checkAgainstDriveLimits)
            {
                var d = ab.xDrive;
                if (Mathf.Abs(d.lowerLimit - lim.lowerDeg) > 0.01f || Mathf.Abs(d.upperLimit - lim.upperDeg) > 0.01f)
                    Debug.LogWarning($"[JointLimitMonitor] {lim.joint}: 표 [{lim.lowerDeg}, {lim.upperDeg}]° vs ArticulationBody xDrive [{d.lowerLimit:F4}, {d.upperLimit:F4}]° (URDF 6.2832 rad = 360.0009° 류의 반올림 차이는 무해)");
            }
        }
        foreach (var name in byName.Keys.Except(limits.Select(l => l.joint)))
            Debug.LogWarning($"[JointLimitMonitor] 로봇 관절 '{name}' 에 한계 표가 없다 — 감시 안 함");

        // CSV
        string logDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "logs"));
        Directory.CreateDirectory(logDir);
        ViolationsCsvPath = Path.Combine(logDir, violationsCsvName);
        NearCsvPath = Path.Combine(logDir, nearCsvName);
        m_Session = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");
        m_Violations = new StreamWriter(ViolationsCsvPath, false);
        m_Violations.WriteLine("session,kind,counted,joint,enter_time_s,exit_time_s,duration_s,peak_value_deg,limit_deg,over_by_deg,samples,ros_stamp_enter");
        m_Violations.Flush();
        m_Near = new StreamWriter(NearCsvPath, false);
        m_Near.WriteLine("session,joint,side,enter_time_s,exit_time_s,duration_s,closest_value_deg,limit_deg,remaining_deg_min,samples,ros_stamp_enter");
        m_Near.Flush();
        Debug.Log($"[JointLimitMonitor] 감시 {m_Tracked.Count}개 관절, 여유 0 (D-3), 근처 {nearMarginDeg}°, 판정 값 {ValueSource}. CSV: {ViolationsCsvPath}, {NearCsvPath}");
    }

    long m_LastSeenMessages;        // 첫 메시지 감지용
    int m_VelocityHoldSteps;        // 첫 메시지 점프 뒤 속도 판정을 쉬는 스텝 수

    void FixedUpdate()
    {
        double t = Time.fixedTimeAsDouble;
        float dt = Time.fixedDeltaTime;
        string stamp = RosStamp();

        // 미러의 첫 메시지는 시작 자세에서 소스의 현재 값으로 한 스텝에 점프한다(퍼블리셔가 먼저 떠 있으면 수십 도).
        // 로봇 동작이 아니라 미러 고유 현상이므로, 그 직후 속도 창이 다시 찰 때까지 속도 판정을 쉰다. 위치 판정은 계속 한다.
        if (mirror != null && m_LastSeenMessages == 0 && mirror.MessageCount > 0)
        {
            m_VelocityHoldSteps = velocityWindowSteps + 2;
            foreach (var tr in m_Tracked) { tr.histCount = 0; tr.histHead = 0; tr.hasLast = false; }
            Debug.Log($"[JointLimitMonitor] 첫 메시지 수신 — {m_VelocityHoldSteps} 스텝 동안 속도 판정 보류(첫 점프 제외)");
        }
        if (mirror != null) m_LastSeenMessages = mirror.MessageCount;
        bool velocityHold = m_VelocityHoldSteps > 0;
        if (velocityHold) m_VelocityHoldSteps--;

        foreach (var tr in m_Tracked)
        {
            float raw = SourceDeg(tr);
            // ±180° 감싸기 감지·펼치기 (CONTRACT §9-3 두 번째 질문). 한 스텝에 180° 넘게 뛰면 감싼 것으로 본다.
            if (tr.hasLast)
            {
                float jump = raw - tr.lastRawDeg;
                if (jump > 180f) { tr.unwrapOffsetDeg -= 360f; if (!WrapDetected) Debug.LogWarning($"[JointLimitMonitor] {tr.lim.joint}: jointPosition 이 ±180° 에서 감싼다 ({tr.lastRawDeg:F2}→{raw:F2}). 상태 발행 때 펼쳐야 한다"); WrapDetected = true; }
                else if (jump < -180f) { tr.unwrapOffsetDeg += 360f; if (!WrapDetected) Debug.LogWarning($"[JointLimitMonitor] {tr.lim.joint}: jointPosition 이 ±180° 에서 감싼다 ({tr.lastRawDeg:F2}→{raw:F2})"); WrapDetected = true; }
            }
            tr.lastRawDeg = raw; tr.hasLast = true;
            float q = raw + tr.unwrapOffsetDeg;

            // 속도: 링버퍼 가장 오래된 값과의 차분
            tr.history[tr.histHead] = q;
            tr.histHead = (tr.histHead + 1) % tr.history.Length;
            if (tr.histCount < tr.history.Length) tr.histCount++;
            float vel = 0f;
            if (tr.histCount == tr.history.Length)
            {
                float oldest = tr.history[tr.histHead];            // 덮어쓰기 직전 위치 = 가장 오래된 값
                vel = (q - oldest) / (dt * (tr.history.Length - 1));
            }

            var L = tr.lim;
            // 위치 하드 한계 (여유 0)
            Track("position", L.joint, counted: true, exceeding: q > L.upperDeg || q < L.lowerDeg,
                  value: q, limit: q >= 0 ? L.upperDeg : L.lowerDeg, t, stamp, m_Violations);
            // 속도 (첫 메시지 점프 직후 보류 구간은 제외)
            Track("velocity", L.joint, counted: true, exceeding: !velocityHold && Mathf.Abs(vel) > L.velocityDegPerSec,
                  value: Mathf.Abs(vel), limit: L.velocityDegPerSec, t, stamp, m_Violations);
            // 운용 한계 (경고 행)
            if (!float.IsNaN(L.opUpperDeg))
                Track("operational", L.joint, counted: false, exceeding: q > L.opUpperDeg || q < L.opLowerDeg,
                      value: q, limit: q >= 0 ? L.opUpperDeg : L.opLowerDeg, t, stamp, m_Violations);
            // 그리퍼 1.0 rad 초과 (정보 행)
            if (!float.IsNaN(L.infoUpperDeg))
                Track("gripper_over_1rad", L.joint, counted: false, exceeding: q > L.infoUpperDeg,
                      value: q, limit: L.infoUpperDeg, t, stamp, m_Violations);
            // 한계 근처 (안쪽 nearMarginDeg 이내, 위반이 아닌 상태)
            bool nearUpper = q <= L.upperDeg && L.upperDeg - q <= nearMarginDeg;
            bool nearLower = q >= L.lowerDeg && q - L.lowerDeg <= nearMarginDeg;
            TrackNear(L.joint, "upper", nearUpper, q, L.upperDeg, t, stamp);
            TrackNear(L.joint, "lower", nearLower, q, L.lowerDeg, t, stamp);
        }
    }

    // 판정 대상 값.
    //  Mirror(M1): 받은 메시지의 값(= Isaac 측정값). Unity 관절은 PhysX 한계(TGS 32회)에 막혀 한계 밖 값을 못 따라가므로,
    //              Unity 측정값으로 세면 Isaac 의 한계 초과를 놓친다(원본보다 관대). 메시지에 없는 관절만 Unity 값.
    //  Command(M2): Unity 가 실제로 움직인 jointPosition.
    RosMessageTypes.Sensor.JointStateMsg m_IndexedMsg;
    readonly Dictionary<string, int> m_MsgIndex = new Dictionary<string, int>();
    public string ValueSource => mirror != null && mirror.mode == JointStateMirror.ApplyMode.Mirror ? "received_message" : "unity_measured";

    float SourceDeg(Tracked tr)
    {
        if (mirror != null && mirror.mode == JointStateMirror.ApplyMode.Mirror && mirror.LastMessage != null)
        {
            var msg = mirror.LastMessage;
            if (!ReferenceEquals(msg, m_IndexedMsg))
            {
                m_IndexedMsg = msg; m_MsgIndex.Clear();
                int n = Math.Min(msg.name.Length, msg.position.Length);
                for (int i = 0; i < n; i++) m_MsgIndex[msg.name[i]] = i;
            }
            if (m_MsgIndex.TryGetValue(tr.lim.joint, out int k)) return (float)(msg.position[k] * Mathf.Rad2Deg);
        }
        return tr.ab.jointPosition[0] * Mathf.Rad2Deg;
    }

    Episode Find(string kind, string joint, string side = null)
    {
        string key = side == null ? kind : kind + ":" + side;
        foreach (var e in m_Episodes) if (e.kind == key && e.joint == joint) return e;
        var n = new Episode { kind = key, joint = joint };
        m_Episodes.Add(n);
        return n;
    }

    void Track(string kind, string joint, bool counted, bool exceeding, float value, float limit, double t, string stamp, StreamWriter w)
    {
        var e = Find(kind, joint);
        e.counted = counted; e.writer = w;
        if (exceeding)
        {
            if (!e.active) { e.active = true; e.enterT = t; e.peak = value; e.limit = limit; e.samples = 0; e.rosStampEnter = stamp; }
            if (Mathf.Abs(value - limit) > Mathf.Abs(e.peak - e.limit)) { e.peak = value; e.limit = limit; }
            e.samples++;
        }
        else if (e.active) CloseEpisode(e, t);
    }

    void CloseEpisode(Episode e, double t)
    {
        e.active = false; e.exitT = t;
        float over = Mathf.Abs(e.peak - e.limit);
        e.writer.WriteLine(string.Join(",", m_Session, e.kind, e.counted ? "1" : "0", e.joint,
            F(e.enterT), F(e.exitT), F(e.exitT - e.enterT), F(e.peak), F(e.limit), F(over), e.samples.ToString(), e.rosStampEnter));
        e.writer.Flush();
        if (e.counted) ViolationCount++;
        else if (e.kind == "operational") OperationalWarnCount++;
        Debug.LogWarning($"[JointLimitMonitor] {(e.counted ? "VIOLATION" : "note")} {e.kind} {e.joint}: peak {e.peak:F3} vs limit {e.limit:F3} (+{over:F3}), {e.exitT - e.enterT:F2}s, {e.samples} steps");
    }

    void TrackNear(string joint, string side, bool near, float q, float limit, double t, string stamp)
    {
        var e = Find("near", joint, side);
        if (near)
        {
            if (!e.active) { e.active = true; e.enterT = t; e.peak = q; e.limit = limit; e.samples = 0; e.rosStampEnter = stamp; }
            if (Mathf.Abs(limit - q) < Mathf.Abs(limit - e.peak)) e.peak = q;   // 한계에 가장 가까웠던 값
            e.samples++;
        }
        else if (e.active)
        {
            e.active = false; e.exitT = t;
            m_Near.WriteLine(string.Join(",", m_Session, e.joint, side, F(e.enterT), F(e.exitT), F(e.exitT - e.enterT), F(e.peak), F(e.limit), F(Mathf.Abs(e.limit - e.peak)), e.samples.ToString(), e.rosStampEnter));
            m_Near.Flush();
            NearCount++;
        }
    }

    string RosStamp()
    {
        var m = mirror != null ? mirror.LastMessage : null;
        if (m == null || m.header == null) return "";
        return ((double)m.header.stamp.sec + m.header.stamp.nanosec * 1e-9).ToString("F3", CultureInfo.InvariantCulture);
    }

    static string F(double v) => v.ToString("F4", CultureInfo.InvariantCulture);

    void OnDestroy()
    {
        double t = Time.fixedTimeAsDouble;
        foreach (var e in m_Episodes.Where(e => e.active).ToList())
        {
            if (e.kind.StartsWith("near")) { e.active = false; e.exitT = t; m_Near.WriteLine(string.Join(",", m_Session, e.joint, e.kind.Substring(5), F(e.enterT), F(e.exitT), F(e.exitT - e.enterT), F(e.peak), F(e.limit), F(Mathf.Abs(e.limit - e.peak)), e.samples.ToString(), e.rosStampEnter)); NearCount++; }
            else CloseEpisode(e, t);
        }
        m_Violations?.Flush(); m_Violations?.Dispose();
        m_Near?.Flush(); m_Near?.Dispose();
        Debug.Log($"[JointLimitMonitor] 종료: 위반 {ViolationCount}, 근처 {NearCount}, 운용 경고 {OperationalWarnCount}, wrap={WrapDetected}");
    }
}
