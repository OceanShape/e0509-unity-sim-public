// 드라이브 강성·감쇠·속도 한계 스윕 (단계 7, D-4 "수렴이 보장되는 값"). 조정용 시험이라 단언하지 않고 표만 남긴다.
// 조합마다 씬을 새로 읽고 Command 모드로:
//   step  — J3 129.89 → 135.0 계단(J2 −94.92 유지): J3 오버슈트, J2 가 한계(−95) 쪽으로 밀린 양, 0.3° 수렴 시간
//   trap  — 120/180 사다리꼴로 스캔 자세에 갔다가 한계 근처 자세로 복귀: 추종 지연, 끝난 뒤 도착 시간, J3 최대·J2 최소
// 결과: logs/m2/drive_sweep.csv
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

public class DriveTuningSweep
{
    static readonly string[] k_CmdNames = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "rh_p12_rn", "rh_r2", "rh_l1", "rh_l2" };
    static readonly string[] k_Arm = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
    static readonly float[] k_Limit = { 87.98f, -94.92f, 135.0f, 175.94f, -31.34f, 93.42f };
    static readonly float[] k_Scan = { 60.0f, -55.0f, 95.0f, 200.0f, -5.0f, 60.0f };
    JointStateMirror m;

    float Meas(string j) => m.Joints[j].jointPosition[0] * Mathf.Rad2Deg;
    float[] Arm() => k_Arm.Select(Meas).ToArray();
    static float MaxErr(float[] a, float[] b) { float e = 0; for (int i = 0; i < a.Length; i++) e = Mathf.Max(e, Mathf.Abs(a[i] - b[i])); return e; }
    void Cmd(float[] arm)
    {
        var p = new double[10];
        for (int i = 0; i < 6; i++) p[i] = arm[i] * Mathf.Deg2Rad;
        for (int i = 6; i < 10; i++) p[i] = 53.05 * Mathf.Deg2Rad;
        m.InjectMessage(new JointStateMsg { name = (string[])k_CmdNames.Clone(), position = p });
    }

    [UnityTest]
    public IEnumerator Sweep()
    {
        var sb = new StringBuilder("stiffness,damping,vel_limit,hold_err,step_J3_overshoot,step_J2_min,step_settle_s,step_max_speed,trap_max_lag,trap_arrival_s,trap_J3_max,trap_J2_min,trap_max_speed,violations\n");
        string path = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "logs", "m2", "drive_sweep.csv"));
        Directory.CreateDirectory(Path.GetDirectoryName(path));
        var ks = new[] { 1e5f, 1e6f, 1e7f };
        var cs = new[] { 1e3f, 1e4f, 1e5f };
        var vls = new[] { true, false };
        try
        {
            foreach (var vl in vls)
            foreach (var k in ks)
            foreach (var c in cs)
            {
                JointStateMirror.ForcedMode = JointStateMirror.ApplyMode.Command;
                JointStateMirror.SkipRos = true;
                JointStateMirror.OverrideStiffness = k; JointStateMirror.OverrideDamping = c; JointStateMirror.OverrideVelocityLimits = vl;
                yield return SceneManager.LoadSceneAsync("Assets/Scenes/E0509Sim.unity", LoadSceneMode.Single);
                yield return null;
                m = UnityEngine.Object.FindObjectOfType<JointStateMirror>();
                var mon = UnityEngine.Object.FindObjectOfType<JointLimitMonitor>();

                // hold 0.5 s
                float hold = 0;
                for (double t0 = Time.fixedTimeAsDouble; Time.fixedTimeAsDouble - t0 < 0.5;) { yield return new WaitForFixedUpdate(); hold = Mathf.Max(hold, MaxErr(Arm(), new[] { 87.98f, -94.92f, 129.89f, 175.94f, -31.34f, 93.42f })); }

                // step J3 → 135
                Cmd(k_Limit);
                float j3o = float.MinValue, j2min = float.MaxValue, spd = 0; double settle = double.NaN; var prev = Arm();
                for (double t0 = Time.fixedTimeAsDouble; Time.fixedTimeAsDouble - t0 < 1.5;)
                {
                    yield return new WaitForFixedUpdate();
                    var a = Arm();
                    j3o = Mathf.Max(j3o, a[2] - 135.0f); j2min = Mathf.Min(j2min, a[1]);
                    for (int i = 0; i < 6; i++) spd = Mathf.Max(spd, Mathf.Abs(a[i] - prev[i]) / Time.fixedDeltaTime);
                    prev = a;
                    if (double.IsNaN(settle) && MaxErr(a, k_Limit) <= 0.3f) settle = Time.fixedTimeAsDouble - t0;
                }

                // trapezoid out & back (120/180)
                float lag = 0, tj3 = float.MinValue, tj2 = float.MaxValue, tspd = 0; double arrive = double.NaN;
                foreach (var (from, to) in new[] { (k_Limit, k_Scan), (k_Scan, k_Limit) })
                {
                    float maxD = 0; for (int i = 0; i < 6; i++) maxD = Mathf.Max(maxD, Mathf.Abs(to[i] - from[i]));
                    float v = 120, acc = 180; double ta = v / acc, T = 2 * ta + (maxD - v * ta) / v;
                    if (v * ta >= maxD) { ta = Math.Sqrt(maxD / acc); T = 2 * ta; v = (float)(acc * ta); }
                    Func<double, float[]> sp = t =>
                    {
                        double s = t <= 0 ? 0 : t < ta ? 0.5 * acc * t * t : t < T - ta ? 0.5 * acc * ta * ta + v * (t - ta) : t < T ? maxD - 0.5 * acc * (T - t) * (T - t) : maxD;
                        return Enumerable.Range(0, 6).Select(i => (float)(from[i] + (to[i] - from[i]) * s / maxD)).ToArray();
                    };
                    double t0 = Time.fixedTimeAsDouble; prev = Arm();
                    while (Time.fixedTimeAsDouble - t0 < T)
                    {
                        Cmd(sp(Time.fixedTimeAsDouble - t0));
                        yield return new WaitForFixedUpdate();
                        var a = Arm();
                        lag = Mathf.Max(lag, MaxErr(a, sp(Time.fixedTimeAsDouble - t0)));
                        tj3 = Mathf.Max(tj3, a[2]); tj2 = Mathf.Min(tj2, a[1]);
                        for (int i = 0; i < 6; i++) tspd = Mathf.Max(tspd, Mathf.Abs(a[i] - prev[i]) / Time.fixedDeltaTime);
                        prev = a;
                    }
                    Cmd(to);
                    double te = Time.fixedTimeAsDouble; arrive = double.NaN;
                    while (Time.fixedTimeAsDouble - te < 1.5)
                    {
                        yield return new WaitForFixedUpdate();
                        var a = Arm(); tj3 = Mathf.Max(tj3, a[2]); tj2 = Mathf.Min(tj2, a[1]);
                        if (double.IsNaN(arrive) && MaxErr(a, to) <= 0.3f) arrive = Time.fixedTimeAsDouble - te;
                    }
                }
                string row = string.Join(",", k.ToString("G"), c.ToString("G"), vl ? "1" : "0", hold.ToString("F4"), j3o.ToString("F4"), j2min.ToString("F4"),
                    settle.ToString("F3"), spd.ToString("F1"), lag.ToString("F3"), arrive.ToString("F3"), tj3.ToString("F4"), tj2.ToString("F4"), tspd.ToString("F1"), mon.ViolationCount);
                sb.AppendLine(row);
                Debug.Log("[DriveTuningSweep] " + row);
                File.WriteAllText(path, sb.ToString());
            }
        }
        finally { File.WriteAllText(path, sb.ToString()); }
    }

    [TearDown]
    public void TearDown()
    {
        JointStateMirror.ForcedMode = null; JointStateMirror.SkipRos = false;
        JointStateMirror.OverrideStiffness = null; JointStateMirror.OverrideDamping = null; JointStateMirror.OverrideVelocityLimits = null; JointStateMirror.OverrideSolverIterations = null;
    }
}
