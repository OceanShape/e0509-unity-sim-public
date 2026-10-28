// M2 Command 모드 시험 (단계 7, CONTRACT §9-4 D-4). ROS 없이 물리만 돈다(SkipRos). 명령은 ROS 콜백과 같은 경로(InjectMessage)로 넣는다.
//
// D-4: Command 모드는 드라이브 목표만 준다. 2시간 안에 0.3° 수렴이 안 되면 운동학 대입으로 폴백하고 README 에 "도착 판정 미검증".
// 원본보다 관대하면 안 된다: 한계 근처 목표(J3 135.0°, 시작 J2 −94.92°)에서 오버슈트로 한계를 넘으면 Isaac 에 없는 위반이다.
//
// 시험 (각각 위치 위반 0 을 요구):
//   1. Hold      — 시작 자세(overview)를 2 s 유지: 측정이 overview 와 0.3° 안, 움직임 없음
//   2. StepLimit — J3 129.89 → 135.0(한계 0.0003° 앞) 계단 명령, J2 −94.92 유지. 0.3° 수렴 시간, 한계 쪽 오버슈트
//   3. Trapezoid — 가상 제어기 MoveJoint 와 같은 사다리꼴(최대 속도·가속도) 설정점을 매 물리 스텝 흘림. 끝난 뒤 0.3° 도착 시간,
//                  경로 중 추종 지연. 60/90(D-7)·120/180(Isaac 런) 두 속도. 마지막 구간은 J3 를 135.0 으로, J2 를 −94.92 로 되돌린다
//   4. Gripper   — 네 관절 0.926 → 1.08 rad 계단(가상 제어기 닫힘 명령). 한계 1.1 rad 안에서 수렴
// 보고서: logs/m2/command_mode_report.json (+ 스텝별 궤적 logs/m2/command_mode_trace.csv)
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using RosMessageTypes.Sensor;

public class CommandModeTest
{
    const string k_ScenePath = "Assets/Scenes/E0509Sim.unity";
    const float k_ArrivalTolDeg = 0.3f;       // 가상 제어기 도착 판정 (CONTRACT §3-5)
    const float k_MaxSettleSec = 1.0f;        // 계단 명령 뒤 이 안에 0.3° 수렴
    const float k_MaxArrivalAfterEndSec = 0.5f; // 사다리꼴 끝난 뒤 이 안에 0.3° 도착 (가상 제어기 타임아웃 8 s, 정체 판정 0.3 s)

    // 가상 제어기 /joint_command name 순서 (CONTRACT §2-2)
    static readonly string[] k_CmdNames = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "rh_p12_rn", "rh_r2", "rh_l1", "rh_l2" };
    static readonly string[] k_Arm = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
    static readonly float[] k_OverviewDeg = { 87.98f, -94.92f, 129.89f, 175.94f, -31.34f, 93.42f };
    const float k_GripOpenDeg = 53.05f;       // 0.926 rad
    const float k_GripCloseDeg = 61.879f;     // 1.08 rad (가상 제어기 닫힘)

    JointStateMirror m_Mirror;
    JointLimitMonitor m_Monitor;
    JointStatePublisher m_Pub;
    readonly Dictionary<string, object> m_Report = new Dictionary<string, object>();
    readonly StringBuilder m_Trace = new StringBuilder("phase,step,t,joint,command_deg,measured_deg,err_deg\n");
    string m_LogDir;

    [UnitySetUp]
    public IEnumerator SetUp()
    {
        JointStateMirror.ForcedMode = JointStateMirror.ApplyMode.Command;
        JointStateMirror.SkipRos = true;
        m_LogDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "logs", "m2"));
        Directory.CreateDirectory(m_LogDir);
        yield return SceneManager.LoadSceneAsync(k_ScenePath, LoadSceneMode.Single);
        yield return null;
        m_Mirror = UnityEngine.Object.FindObjectOfType<JointStateMirror>();
        m_Monitor = UnityEngine.Object.FindObjectOfType<JointLimitMonitor>();
        m_Pub = UnityEngine.Object.FindObjectOfType<JointStatePublisher>();
        Assert.IsNotNull(m_Mirror); Assert.IsNotNull(m_Monitor);
        Assert.AreEqual(JointStateMirror.ApplyMode.Command, m_Mirror.mode, "ForcedMode 가 적용되지 않았다");
    }

    // 코루틴이 예외로 끝나면 iterator 의 finally 는 돌지 않는다(러너가 Dispose 하지 않음). 보고서는 TearDown 에서 쓴다.
    [TearDown]
    public void TearDown()
    {
        JointStateMirror.ForcedMode = null;
        JointStateMirror.SkipRos = false;
        if (m_Monitor == null || m_LogDir == null) return;
        if (!m_Report.ContainsKey("result")) m_Report["result"] = "FAIL";
        m_Report["violations_total"] = m_Monitor.ViolationCount;
        m_Report["near_limit_episodes"] = m_Monitor.NearCount;
        File.WriteAllText(Path.Combine(m_LogDir, "command_mode_report.json"), TestJson.Write(m_Report));
        File.WriteAllText(Path.Combine(m_LogDir, "command_mode_trace.csv"), m_Trace.ToString());
        if (File.Exists(m_Monitor.ViolationsCsvPath)) File.Copy(m_Monitor.ViolationsCsvPath, Path.Combine(m_LogDir, "command_mode_limit_violations.csv"), true);
        if (File.Exists(m_Monitor.NearCsvPath)) File.Copy(m_Monitor.NearCsvPath, Path.Combine(m_LogDir, "command_mode_limit_near.csv"), true);
        Debug.Log("[CommandModeTest] 보고서 → " + Path.Combine(m_LogDir, "command_mode_report.json"));
    }

    // ── 도우미 ──
    float Meas(string j) => m_Mirror.Joints[j].jointPosition[0] * Mathf.Rad2Deg;
    float[] ArmMeas() => k_Arm.Select(Meas).ToArray();

    void Command(float[] armDeg, float gripDeg)
    {
        var pos = new double[k_CmdNames.Length];
        for (int i = 0; i < 6; i++) pos[i] = armDeg[i] * Mathf.Deg2Rad;
        for (int i = 6; i < 10; i++) pos[i] = gripDeg * Mathf.Deg2Rad;
        m_Mirror.InjectMessage(new JointStateMsg { name = (string[])k_CmdNames.Clone(), position = pos });
    }

    void Trace(string phase, int step, float[] cmdArm, float gripCmd)
    {
        double t = Time.fixedTimeAsDouble;
        for (int i = 0; i < 10; i++)
        {
            string j = k_CmdNames[i];
            float c = i < 6 ? cmdArm[i] : gripCmd;
            float m = Meas(j);
            m_Trace.Append(phase).Append(',').Append(step).Append(',').Append(t.ToString("F4")).Append(',').Append(j).Append(',')
                   .Append(c.ToString("F4")).Append(',').Append(m.ToString("F4")).Append(',').Append(Mathf.Abs(m - c).ToString("F4")).Append('\n');
        }
    }

    static float MaxErr(float[] a, float[] b) { float e = 0; for (int i = 0; i < a.Length; i++) e = Mathf.Max(e, Mathf.Abs(a[i] - b[i])); return e; }

    int PositionViolations() => m_Monitor.ViolationCount;   // counted 위반(위치+속도). 속도는 따로 보고

    // 사다리꼴 프로파일: 동기화된 관절 이동(가장 먼 관절 기준), 최대 속도 v, 가속도 a (deg/s, deg/s²)
    static Func<double, float[]> Trapezoid(float[] from, float[] to, float v, float a, out double duration)
    {
        float maxD = 0; for (int i = 0; i < from.Length; i++) maxD = Mathf.Max(maxD, Mathf.Abs(to[i] - from[i]));
        double ta = v / a, da = 0.5 * a * ta * ta, T;
        if (2 * da >= maxD) { ta = Math.Sqrt(maxD / a); T = 2 * ta; v = (float)(a * ta); }
        else T = 2 * ta + (maxD - 2 * da) / v;
        duration = T;
        double taF = ta, TF = T; float vF = v;
        return t =>
        {
            double s;   // 0..maxD 진행 거리
            if (t <= 0) s = 0;
            else if (t < taF) s = 0.5 * a * t * t;
            else if (t < TF - taF) s = 0.5 * a * taF * taF + vF * (t - taF);
            else if (t < TF) { double r = TF - t; s = maxD - 0.5 * a * r * r; }
            else s = maxD;
            double k = maxD > 0 ? s / maxD : 1;
            var p = new float[from.Length];
            for (int i = 0; i < from.Length; i++) p[i] = (float)(from[i] + (to[i] - from[i]) * k);
            return p;
        };
    }

    [UnityTest]
    public IEnumerator CommandMode_ConvergesWithinTolerance_WithoutLimitViolations()
    {
        var phases = new List<object>();
        m_Report["phases"] = phases;            // 참조를 먼저 넣어 둔다(실패해도 TearDown 보고서에 남게)
        m_Report["started"] = DateTime.Now.ToString("s");
        var root = m_Mirror.Joints.Values.First().GetComponentsInParent<ArticulationBody>().Last();
        m_Report["solver"] = $"{UnityEngine.Physics.defaultSolverIterations}/{UnityEngine.Physics.defaultSolverVelocityIterations} default, root {root.name} {root.solverIterations}/{root.solverVelocityIterations}";
        m_Report["fixed_delta_time"] = Time.fixedDeltaTime;
        m_Report["max_joint_velocity_rad_s"] = m_Mirror.Joints.ToDictionary(kv => kv.Key, kv => (object)kv.Value.maxJointVelocity);
        m_Report["drive"] = m_Mirror.Joints.ToDictionary(kv => kv.Key, kv => (object)$"k={kv.Value.xDrive.stiffness:G} c={kv.Value.xDrive.damping:G} f={kv.Value.xDrive.forceLimit:G}");
        m_Report["publisher_active"] = m_Pub != null && m_Pub.Active;
        try
        {
            // ── 1. Hold ──
            var hold = new Dictionary<string, object> { ["phase"] = "hold" };
            float holdMax = 0; int step = 0;
            for (double t0 = Time.fixedTimeAsDouble; Time.fixedTimeAsDouble - t0 < 2.0; step++)
            {
                yield return new WaitForFixedUpdate();
                holdMax = Mathf.Max(holdMax, MaxErr(ArmMeas(), k_OverviewDeg));
                Trace("hold", step, k_OverviewDeg, k_GripOpenDeg);
            }
            hold["max_err_deg"] = holdMax;
            hold["violations"] = PositionViolations();
            phases.Add(hold);
            Assert.LessOrEqual(holdMax, k_ArrivalTolDeg, $"시작 자세 유지 실패: 최대 오차 {holdMax:F3}°");
            Assert.AreEqual(0, PositionViolations(), "시작 자세 유지 중 위반");

            // ── 2. StepLimit ──
            // 2a. 5° 계단(129.89 → 135.0): 가상 제어기는 팔에 이런 큰 계단을 주지 않는다(MoveJoint 5 ms 보간, MoveSplineJoint 2° 간격,
            //     CONTRACT §3-2). 민감도 기록용이라 단언하지 않는다.
            yield return StepPhase("step5_J3_129.89_to_135_info", new[] { 87.98f, -94.92f, 135.0f, 175.94f, -31.34f, 93.42f }, k_GripOpenDeg, phases, assertPass: false);
            yield return StepPhase("reset_J3_to_133", new[] { 87.98f, -94.92f, 133.0f, 175.94f, -31.34f, 93.42f }, k_GripOpenDeg, phases, assertPass: false);
            // 2b. 2° 계단(133 → 135.0): 가상 제어기가 팔에 주는 가장 큰 설정점 간격. 0.3° 수렴·위반 0 을 요구한다.
            yield return StepPhase("step2_J3_133_to_135", new[] { 87.98f, -94.92f, 135.0f, 175.94f, -31.34f, 93.42f }, k_GripOpenDeg, phases);

            // ── 3. Trapezoid: 60/90, 120/180. 스캔 자세 쪽으로 갔다가 한계 근처 자세로 돌아온다 ──
            var scan = new[] { 60.0f, -55.0f, 95.0f, 200.0f, -5.0f, 60.0f };
            var limitPose = new[] { 87.98f, -94.92f, 135.0f, 175.94f, -31.34f, 93.42f };
            foreach (var (v, a) in new[] { (60f, 90f), (120f, 180f) })
            {
                yield return TrapezoidPhase($"trap_{v:F0}_{a:F0}_out", limitPose, scan, v, a, phases);
                yield return TrapezoidPhase($"trap_{v:F0}_{a:F0}_back_to_limit_pose", scan, limitPose, v, a, phases);
            }

            // ── 3b. 속도 한계 정보: J1 을 한 번에 +60° 계단 → 측정 최고 속도를 maxJointVelocity(180 °/s)와 비교. 단언하지 않는다 ──
            {
                var big = (float[])limitPose.Clone(); big[0] += 60f;
                var prevJ1 = Meas("joint_1"); float peak = 0;
                Command(big, k_GripOpenDeg);
                for (double t0 = Time.fixedTimeAsDouble; Time.fixedTimeAsDouble - t0 < 1.5;)
                {
                    yield return new WaitForFixedUpdate();
                    float j1 = Meas("joint_1");
                    peak = Mathf.Max(peak, Mathf.Abs(j1 - prevJ1) / Time.fixedDeltaTime);
                    prevJ1 = j1;
                }
                phases.Add(new Dictionary<string, object> { ["phase"] = "info_J1_step60_peak_speed", ["peak_deg_s"] = peak,
                    ["limit_deg_s"] = 180f, ["max_joint_velocity_rad_s"] = m_Mirror.Joints["joint_1"].maxJointVelocity, ["asserted"] = false });
                Command(limitPose, k_GripOpenDeg);
                for (double t0 = Time.fixedTimeAsDouble; Time.fixedTimeAsDouble - t0 < 1.5;) yield return new WaitForFixedUpdate();
            }

            // ── 4. Gripper close ──
            yield return StepPhase("gripper_close_1.08rad", limitPose, k_GripCloseDeg, phases);

            m_Report["phases"] = phases;
            m_Report["violations_total"] = m_Monitor.ViolationCount;
            m_Report["near_limit_episodes"] = m_Monitor.NearCount;
            m_Report["violations_csv"] = m_Monitor.ViolationsCsvPath;
            m_Report["result"] = "PASS";
        }
        finally { }
    }

    // 계단 명령 → 0.3° 수렴 시간, 목표 너머로 간 최대량(오버슈트), 위반
    IEnumerator StepPhase(string name, float[] armTarget, float gripTarget, List<object> phases, bool assertPass = true)
    {
        int v0 = PositionViolations();
        Command(armTarget, gripTarget);
        double t0 = Time.fixedTimeAsDouble;
        double settled = double.NaN;
        var start = ArmMeas();
        float overshoot = 0, maxVel = 0; var prev = ArmMeas();
        int step = 0;
        while (Time.fixedTimeAsDouble - t0 < 2.0)
        {
            yield return new WaitForFixedUpdate();
            var m = ArmMeas();
            float err = MaxErr(m, armTarget);
            float gErr = Mathf.Abs(Meas("rh_p12_rn") - gripTarget);
            for (int i = 0; i < 6; i++)
            {
                float dir = Mathf.Sign(armTarget[i] - start[i]);
                if (dir != 0) overshoot = Mathf.Max(overshoot, (m[i] - armTarget[i]) * dir);
                maxVel = Mathf.Max(maxVel, Mathf.Abs(m[i] - prev[i]) / Time.fixedDeltaTime);
            }
            prev = m;
            if (double.IsNaN(settled) && err <= k_ArrivalTolDeg && gErr <= k_ArrivalTolDeg) settled = Time.fixedTimeAsDouble - t0;
            Trace(name, step++, armTarget, gripTarget);
        }
        var final = ArmMeas();
        var r = new Dictionary<string, object>
        {
            ["phase"] = name,
            ["settle_sec"] = settled,
            ["final_err_deg"] = MaxErr(final, armTarget),
            ["gripper_final_err_deg"] = Mathf.Abs(Meas("rh_p12_rn") - gripTarget),
            ["overshoot_deg"] = overshoot,
            ["max_arm_speed_deg_s"] = maxVel,
            ["J2_final_deg"] = final[1], ["J3_final_deg"] = final[2],
            ["new_violations"] = PositionViolations() - v0,
        };
        r["asserted"] = assertPass;
        phases.Add(r);
        if (!assertPass) yield break;
        Assert.IsFalse(double.IsNaN(settled), $"{name}: 2 s 안에 {k_ArrivalTolDeg}° 로 수렴하지 않음 (최종 오차 {MaxErr(final, armTarget):F3}°)");
        Assert.LessOrEqual((float)settled, k_MaxSettleSec, $"{name}: 수렴 {settled:F3}s > {k_MaxSettleSec}s");
        Assert.AreEqual(0, PositionViolations() - v0, $"{name}: 한계 위반 {PositionViolations() - v0}건");
    }

    // 사다리꼴 설정점 스트림 → 경로 중 최대 추종 지연, 끝난 뒤 0.3° 도착 시간, 위반
    IEnumerator TrapezoidPhase(string name, float[] from, float[] to, float v, float a, List<object> phases)
    {
        int v0 = PositionViolations();
        var prof = Trapezoid(from, to, v, a, out double T);
        double t0 = Time.fixedTimeAsDouble;
        float maxLag = 0, maxVel = 0; var prev = ArmMeas();
        int step = 0;
        while (Time.fixedTimeAsDouble - t0 < T)
        {
            var sp = prof(Time.fixedTimeAsDouble - t0);
            Command(sp, k_GripOpenDeg);
            yield return new WaitForFixedUpdate();
            var m = ArmMeas();
            maxLag = Mathf.Max(maxLag, MaxErr(m, prof(Time.fixedTimeAsDouble - t0)));
            for (int i = 0; i < 6; i++) maxVel = Mathf.Max(maxVel, Mathf.Abs(m[i] - prev[i]) / Time.fixedDeltaTime);
            prev = m;
            Trace(name, step++, sp, k_GripOpenDeg);
        }
        Command(to, k_GripOpenDeg);
        double tEnd = Time.fixedTimeAsDouble, arrived = double.NaN;
        float overshoot = 0;
        while (Time.fixedTimeAsDouble - tEnd < 2.0)
        {
            yield return new WaitForFixedUpdate();
            var m = ArmMeas();
            for (int i = 0; i < 6; i++) { float dir = Mathf.Sign(to[i] - from[i]); if (dir != 0) overshoot = Mathf.Max(overshoot, (m[i] - to[i]) * dir); }
            if (double.IsNaN(arrived) && MaxErr(m, to) <= k_ArrivalTolDeg) arrived = Time.fixedTimeAsDouble - tEnd;
            Trace(name + "_settle", step++, to, k_GripOpenDeg);
            if (!double.IsNaN(arrived) && Time.fixedTimeAsDouble - tEnd > arrived + 0.5) break;
        }
        var final = ArmMeas();
        phases.Add(new Dictionary<string, object>
        {
            ["phase"] = name, ["vel_deg_s"] = v, ["acc_deg_s2"] = a, ["profile_sec"] = T,
            ["max_tracking_lag_deg"] = maxLag, ["max_arm_speed_deg_s"] = maxVel,
            ["arrival_after_end_sec"] = arrived, ["final_err_deg"] = MaxErr(final, to), ["overshoot_deg"] = overshoot,
            ["J2_final_deg"] = final[1], ["J3_final_deg"] = final[2],
            ["new_violations"] = PositionViolations() - v0,
        });
        Assert.IsFalse(double.IsNaN(arrived), $"{name}: 끝난 뒤 2 s 안에 도착하지 않음 (최종 오차 {MaxErr(final, to):F3}°)");
        Assert.LessOrEqual((float)arrived, k_MaxArrivalAfterEndSec, $"{name}: 도착 {arrived:F3}s > {k_MaxArrivalAfterEndSec}s");
        Assert.AreEqual(0, PositionViolations() - v0, $"{name}: 한계 위반 {PositionViolations() - v0}건");
    }
}
