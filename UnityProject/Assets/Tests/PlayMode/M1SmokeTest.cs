// M1 헤드리스 스모크 테스트 (단계 6). Unity Test Framework PlayMode, batchmode 에서 실행.
//
//   bash scripts/run_m1.sh fake --hold 1 --ramp 1 --limit-test &      (도메인은 무엇이든 — Unity 는 TCP 로만 붙는다)
//   bash scripts/unity_test_m1.sh                                        (이 테스트를 -runTests 로 돌리고 결과·보고서를 모은다)
//
// 확인하는 것 (단계 6 의 눈 확인을 수치로 대신한다):
//   1. ROS-TCP 연결이 CONNECTED 가 된다 (엔드포인트 127.0.0.1:10000)
//   2. /dsr01/joint_states 수신 Hz ≥ 50 (가짜 퍼블리셔 60 Hz)
//   3. 6축이 실제로 움직인다 (측정 jointPosition 의 범위 > 5°)
//   4. Mirror 모드: 명령(target)과 측정(jointPosition) 차이가 0.3° 안 (jointPosition 대입이 적용되는지)
//   5. logs/limit_violations.csv·limit_near.csv 가 생기고, --limit-test 면 J3 위반이 1건 이상 센다
//   6. 정보: jointPosition 이 ±180° 에서 감싸는가 (CONTRACT §9-3), TCP 궤적 점 수
//   7. 잡은 과실 따라가기(GraspFollower): bag 을 /sim/grasp_event 와 함께 틀고 M1_EXPECT_GRASPS=N 이면
//      ATTACH·RELEASE 가 N 번씩, 무시 0, 수확된 과실 N 개가 모두 처음 자리에서 5 cm 넘게 옮겨졌다
// 보고서: logs/m1/smoke_report.json (unity_test_m1.sh 가 STAMP 폴더로 옮긴다)
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

public class M1SmokeTest
{
    const string k_ScenePath = "Assets/Scenes/E0509Sim.unity";
    const float k_ConnectTimeout = 20f;     // 엔드포인트 접속 + 첫 메시지
    // 관찰 구간: 기본 8 s(사인파 주기). bag 재생은 M1_OBSERVE_SECONDS 로 런 전체를 본다
    static float ObserveSeconds => float.TryParse(Environment.GetEnvironmentVariable("M1_OBSERVE_SECONDS"), out var v) ? v : 8f;
    const float k_MinHz = 50f;
    const float k_MinMotionDeg = 5f;
    const float k_MirrorTolDeg = 0.3f;      // D-4 의 수렴 기준을 Mirror 모드 대입 검증에 그대로 적용

    [UnityTest, Timeout(1200000)]   // bag 재생 관찰(수 분)이 UTF 기본 타임아웃보다 길다
    public IEnumerator Mirror_FollowsFakeJointStates()
    {
        var report = new Dictionary<string, object> { ["scene"] = k_ScenePath, ["started"] = DateTime.Now.ToString("s") };
        string logDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "logs"));
        Directory.CreateDirectory(Path.Combine(logDir, "m1"));
        string reportPath = Path.Combine(logDir, "m1", "smoke_report.json");

        try
        {
            // 씬이 실수로 Command 로 저장돼도 M1 은 Mirror 로 돈다(Command 면 이 테스트가 같은 토픽을 발행해 자기 출력을 받게 된다)
            JointStateMirror.ForcedMode = JointStateMirror.ApplyMode.Mirror;
            yield return SceneManager.LoadSceneAsync(k_ScenePath, LoadSceneMode.Single);
            yield return null;

            var mirror = UnityEngine.Object.FindObjectOfType<JointStateMirror>();
            var monitor = UnityEngine.Object.FindObjectOfType<JointLimitMonitor>();
            var trail = UnityEngine.Object.FindObjectOfType<TcpTrail>();
            Assert.IsNotNull(mirror, "JointStateMirror 가 씬에 없다");
            Assert.IsNotNull(monitor, "JointLimitMonitor 가 씬에 없다");
            report["mode"] = mirror.mode.ToString();
            report["topic"] = mirror.topic;
            report["joints_mapped"] = mirror.Joints.Keys.ToArray();
            Assert.AreEqual(10, mirror.Joints.Count, "매핑된 비고정 관절은 10개여야 한다");

            // 1. 연결 + 첫 메시지
            float t0 = Time.realtimeSinceStartup;
            while (Time.realtimeSinceStartup - t0 < k_ConnectTimeout && !(mirror.Connected && mirror.MessageCount > 0))
                yield return null;
            report["connect_seconds"] = Time.realtimeSinceStartup - t0;
            report["connected"] = mirror.Connected;
            report["ros_ip_port"] = mirror.Ros != null ? $"{mirror.Ros.RosIPAddress}:{mirror.Ros.RosPort}" : null;
            Assert.IsTrue(mirror.Connected, "ROS-TCP 연결 실패 (엔드포인트 127.0.0.1:10000 이 떠 있나?)");
            Assert.Greater(mirror.MessageCount, 0, "메시지 수신 없음 (가짜 퍼블리셔가 /dsr01/joint_states 를 내나?)");

            // 2–4. 관찰: Hz, 관절별 측정 범위, 명령-측정 오차, 수신 주기
            var minDeg = new Dictionary<string, float>(); var maxDeg = new Dictionary<string, float>(); var maxErr = new Dictionary<string, float>();
            foreach (var k in mirror.Joints.Keys) { minDeg[k] = float.MaxValue; maxDeg[k] = float.MinValue; maxErr[k] = 0f; }
            var hzSamples = new List<float>();
            var maxErrAt = new Dictionary<string, string>();          // 최대 오차가 난 스텝의 (t, target, measured, msgs)
            var trace = new System.Text.StringBuilder("step,t_fixed,msgs,joint,target_deg,target_clamped_deg,measured_deg,err_deg\n");
            long msgs0 = mirror.MessageCount;
            float obs0 = Time.realtimeSinceStartup;
            int fixedSteps = 0;
            int stepsOutOfLimits = 0; float maxErrWhileOut = 0f;
            int traceEvery = ObserveSeconds > 30f ? 6 : 1;   // 긴 관찰(bag)은 0.1 s 마다만 궤적을 남긴다
            report["observe_seconds_requested"] = ObserveSeconds;
            report["trace_every_steps"] = traceEvery;
            while (Time.realtimeSinceStartup - obs0 < ObserveSeconds)
            {
                yield return new WaitForFixedUpdate();
                fixedSteps++;
                // 어느 관절이든 한계 밖으로 명령된 스텝(--limit-test 의 J3 137°)은 추종 오차 판정에서 뺀다:
                // PhysX 가 그 관절의 한계 위반을 투영 보정하는 동안 이웃 관절까지 0.2–0.5° 오프셋이 생긴다(trace 20:26, 스텝 279–308).
                // Isaac 은 한계 밖 값을 내지 않으므로 실제 런에는 없는 상황이고, 한계 초과 자체는 JointLimitMonitor 가 센다.
                bool anyOut = false;
                foreach (var kv in mirror.Joints) { var dd = kv.Value.xDrive; if (dd.target > dd.upperLimit || dd.target < dd.lowerLimit) anyOut = true; }
                if (anyOut) stepsOutOfLimits++;
                foreach (var kv in mirror.Joints)
                {
                    var d = kv.Value.xDrive;
                    float meas = kv.Value.jointPosition[0] * Mathf.Rad2Deg;
                    float targ = Mathf.Clamp(d.target, d.lowerLimit, d.upperLimit);
                    minDeg[kv.Key] = Mathf.Min(minDeg[kv.Key], meas);
                    maxDeg[kv.Key] = Mathf.Max(maxDeg[kv.Key], meas);
                    // 감싸기(±180°)로 생기는 360° 차이는 오차로 보지 않는다
                    float err = Mathf.Abs(Mathf.DeltaAngle(meas, targ));
                    if (anyOut) { maxErrWhileOut = Mathf.Max(maxErrWhileOut, err); }
                    else if (err > maxErr[kv.Key])
                    {
                        maxErr[kv.Key] = err;
                        maxErrAt[kv.Key] = $"step {fixedSteps} t {Time.fixedTimeAsDouble:F3} msgs {mirror.MessageCount} target {d.target:F3} measured {meas:F3}";
                    }
                    if (fixedSteps % traceEvery == 0) trace.Append(fixedSteps).Append(',').Append(Time.fixedTimeAsDouble.ToString("F4")).Append(',').Append(mirror.MessageCount).Append(',')
                         .Append(kv.Key).Append(',').Append(d.target.ToString("F4")).Append(',').Append(targ.ToString("F4")).Append(',')
                         .Append(meas.ToString("F4")).Append(',').Append(err.ToString("F4")).Append('\n');
                }
                if (Time.realtimeSinceStartup - obs0 > 1f) hzSamples.Add(mirror.ReceiveHz);
            }
            File.WriteAllText(Path.Combine(logDir, "m1", "mirror_trace.csv"), trace.ToString());
            report["max_err_at"] = maxErrAt;
            report["steps_with_command_out_of_limits"] = stepsOutOfLimits;
            report["max_err_deg_while_out_of_limits"] = maxErrWhileOut;
            float obsDur = Time.realtimeSinceStartup - obs0;
            float avgHzByCount = (mirror.MessageCount - msgs0) / obsDur;
            report["observe_seconds"] = obsDur;
            report["fixed_steps"] = fixedSteps;
            report["fixed_delta_time"] = Time.fixedDeltaTime;
            report["messages_in_window"] = mirror.MessageCount - msgs0;
            report["hz_by_count"] = avgHzByCount;
            report["hz_window_min"] = hzSamples.Count > 0 ? hzSamples.Min() : 0f;
            report["hz_window_max"] = hzSamples.Count > 0 ? hzSamples.Max() : 0f;
            report["unknown_names"] = mirror.UnknownNames.ToArray();
            var motion = mirror.Joints.Keys.ToDictionary(k => k, k => maxDeg[k] - minDeg[k]);
            report["motion_range_deg"] = motion;
            report["max_abs_err_deg_clamped_target_vs_measured"] = maxErr;
            report["measured_min_deg"] = minDeg;
            report["measured_max_deg"] = maxDeg;

            Assert.GreaterOrEqual(avgHzByCount, k_MinHz, $"수신 Hz {avgHzByCount:F1} < {k_MinHz}");
            foreach (var j in new[] { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" })
                Assert.Greater(motion[j], k_MinMotionDeg, $"{j} 가 거의 움직이지 않음 (범위 {motion[j]:F2}°)");
            Assert.AreEqual(0, mirror.UnknownNames.Count, "매핑에 없는 관절 이름이 왔다: " + string.Join(",", mirror.UnknownNames));
            if (mirror.mode == JointStateMirror.ApplyMode.Mirror)
                foreach (var kv in maxErr)
                    Assert.LessOrEqual(kv.Value, k_MirrorTolDeg, $"Mirror 모드인데 {kv.Key} 명령-측정 차이 {kv.Value:F3}° > {k_MirrorTolDeg}°");

            // 5. 한계 모니터 CSV
            report["violations"] = monitor.ViolationCount;
            report["near_limit"] = monitor.NearCount;
            report["operational_warn"] = monitor.OperationalWarnCount;
            report["active_violations"] = monitor.ActiveViolationJoints;
            report["violations_csv"] = monitor.ViolationsCsvPath;
            report["near_csv"] = monitor.NearCsvPath;
            report["monitor_value_source"] = monitor.ValueSource;
            // 받은 값이 한계를 넘은 스텝이 있었으면 모니터가 위반으로 세야 한다. Unity 관절은 PhysX 한계에 막히므로
            // Unity 측정값만 보면 놓친다(단계 7에서 TGS 32회로 바꾼 뒤 실제로 놓쳤다).
            if (stepsOutOfLimits > 0)
                Assert.GreaterOrEqual(monitor.ViolationCount, 1, $"받은 값이 한계를 넘은 스텝 {stepsOutOfLimits}개인데 모니터 위반 0건");
            Assert.IsTrue(File.Exists(monitor.ViolationsCsvPath), "limit_violations.csv 없음");
            Assert.IsTrue(File.Exists(monitor.NearCsvPath), "limit_near.csv 없음");

            // 7. 잡은 과실 따라가기
            var follower = UnityEngine.Object.FindObjectOfType<GraspFollower>();
            if (follower != null)
            {
                report["grasp_attach"] = follower.AttachCount;
                report["grasp_release"] = follower.ReleaseCount;
                report["grasp_ignored"] = follower.IgnoredCount;
                report["grasp_placed_in_tray"] = follower.PlacedCount;
                report["grasp_dropped_outside"] = follower.DroppedCount;
                report["grasp_resets"] = follower.ResetCount;
                var moved = new Dictionary<string, float>();
                foreach (var fr in follower.Fruits)
                    if (fr.t != null && fr.harvested) moved[fr.name + " " + fr.outcome] = Vector3.Distance(fr.t.position, fr.homeParent.TransformPoint(fr.homeLocalPos)) * 1000f;
                report["grasp_moved_mm"] = moved;
                report["grasp_events"] = follower.EventLog.ToArray();
                if (int.TryParse(Environment.GetEnvironmentVariable("M1_EXPECT_GRASPS"), out var ng))
                {
                    Assert.AreEqual(ng, follower.AttachCount, "ATTACH 수");
                    Assert.AreEqual(ng, follower.ReleaseCount, "RELEASE 수");
                    Assert.AreEqual(0, follower.IgnoredCount, "무시된 grasp 이벤트: " + string.Join(" | ", follower.EventLog));
                    Assert.AreEqual(ng, moved.Count, "수확된 과실 수");
                    foreach (var kv in moved) Assert.Greater(kv.Value, 50f, $"{kv.Key} 가 그리퍼를 따라 옮겨지지 않았다");
                }
            }
            else if (Environment.GetEnvironmentVariable("M1_EXPECT_GRASPS") != null) Assert.Fail("GraspFollower 가 씬에 없다");

            // 6. 정보
            report["wrap_detected"] = monitor.WrapDetected;
            report["tcp_points"] = trail != null ? trail.PointCount : -1;
            report["tcp_world"] = trail != null ? trail.TcpWorldPosition.ToString("F4") : null;
            report["result"] = "PASS";
        }
        finally
        {
            File.WriteAllText(reportPath, ToJson(report));
            Debug.Log("[M1SmokeTest] 보고서 → " + reportPath);
        }
    }

    [TearDown]
    public void TearDown() { JointStateMirror.ForcedMode = null; }

    // 작은 JSON 직렬화 (Newtonsoft 의존 없이). 값: string/bool/number/배열/사전.
    static string ToJson(object v, int indent = 0)
    {
        string pad = new string(' ', indent);
        switch (v)
        {
            case null: return "null";
            case string s: return "\"" + s.Replace("\\", "\\\\").Replace("\"", "\\\"") + "\"";
            case bool b: return b ? "true" : "false";
            case float f: return f.ToString("R", System.Globalization.CultureInfo.InvariantCulture);
            case double d: return d.ToString("R", System.Globalization.CultureInfo.InvariantCulture);
            case int i: return i.ToString();
            case long l: return l.ToString();
            case IDictionary<string, object> dict:
                return "{\n" + string.Join(",\n", dict.Select(kv => $"{pad}  {ToJson(kv.Key)}: {ToJson(kv.Value, indent + 2)}")) + "\n" + pad + "}";
            case IDictionary<string, float> fd:
                return "{\n" + string.Join(",\n", fd.Select(kv => $"{pad}  {ToJson(kv.Key)}: {ToJson(kv.Value)}")) + "\n" + pad + "}";
            case IDictionary<string, string> sd:
                return "{\n" + string.Join(",\n", sd.Select(kv => $"{pad}  {ToJson(kv.Key)}: {ToJson(kv.Value)}")) + "\n" + pad + "}";
            case IEnumerable<string> arr: return "[" + string.Join(", ", arr.Select(x => ToJson(x))) + "]";
            default: return ToJson(v.ToString());
        }
    }
}
