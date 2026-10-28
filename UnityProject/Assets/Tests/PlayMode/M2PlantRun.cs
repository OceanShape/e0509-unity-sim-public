// M2 헤드리스 플랜트 (단계 7). scripts/run_m2_headless.sh 가 이 테스트를 batchmode 로 띄워 Unity 를 플랜트로 쓴다.
// Command 모드 + ROS 연결: /joint_command 를 드라이브 목표로 적용하고 /dsr01/joint_states·/isaac_sim/strawberries 를 낸다.
// 판정은 하지 않는다(가상 제어기·스캔 로그로 한다, CONTRACT §6). 이 테스트는 플랜트가 살아 있었다는 기록만 남긴다.
//
// 끝나는 조건: 환경 변수 M2_STOP_FILE 의 파일이 생기거나, M2_MAX_SECONDS(기본 1500 s)가 지나면.
// 보고서: $M2_LOG_DIR/unity_plant_report.json (잡은 과실 따라가기 이벤트 포함), 한계 모니터 CSV 사본.
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

public class M2PlantRun
{
    readonly Dictionary<string, object> m_Report = new Dictionary<string, object>();
    JointStateMirror m_Mirror; JointLimitMonitor m_Monitor; JointStatePublisher m_Pub; StrawberryPublisher m_Berry;
    string m_OutDir;

    [UnityTest, Timeout(1800000)]   // 30 분. UTF 기본 타임아웃(수 분)보다 길게
    public IEnumerator Plant()
    {
        if (Environment.GetEnvironmentVariable("M2_STOP_FILE") == null)
            Assert.Ignore("M2_STOP_FILE 이 없다 — run_m2_headless.sh 로만 실행한다");
        string stopFile = Environment.GetEnvironmentVariable("M2_STOP_FILE");
        double maxSec = double.TryParse(Environment.GetEnvironmentVariable("M2_MAX_SECONDS"), out var ms) ? ms : 1500;
        m_OutDir = Environment.GetEnvironmentVariable("M2_LOG_DIR") ?? Path.GetDirectoryName(stopFile);

        JointStateMirror.ForcedMode = JointStateMirror.ApplyMode.Command;
        JointStateMirror.SkipRos = false;
        yield return SceneManager.LoadSceneAsync("Assets/Scenes/E0509Sim.unity", LoadSceneMode.Single);
        yield return null;
        m_Mirror = UnityEngine.Object.FindObjectOfType<JointStateMirror>();
        m_Monitor = UnityEngine.Object.FindObjectOfType<JointLimitMonitor>();
        m_Pub = UnityEngine.Object.FindObjectOfType<JointStatePublisher>();
        m_Berry = UnityEngine.Object.FindObjectOfType<StrawberryPublisher>();
        m_Report["started"] = DateTime.Now.ToString("s");
        m_Report["mode"] = m_Mirror.mode.ToString();
        m_Report["subscribed"] = m_Mirror.topic;
        Debug.Log($"[M2PlantRun] 플랜트 시작: mode={m_Mirror.mode}, 정지 파일 {stopFile}, 최대 {maxSec:F0} s");

        double t0 = Time.realtimeSinceStartupAsDouble, lastLog = t0;
        var hzSamples = new List<float>();
        while (!File.Exists(stopFile) && Time.realtimeSinceStartupAsDouble - t0 < maxSec)
        {
            yield return null;
            double now = Time.realtimeSinceStartupAsDouble;
            if (now - lastLog >= 10.0)
            {
                lastLog = now;
                hzSamples.Add(m_Pub != null ? m_Pub.PublishHz : 0f);
                Debug.Log($"[M2PlantRun] t={now - t0:F0}s connected={m_Mirror.Connected} 명령 {m_Mirror.MessageCount} 상태 발행 {m_Pub?.PublishCount} ({m_Pub?.PublishHz:F0} Hz) 위반 {m_Monitor.ViolationCount} 근처 {m_Monitor.NearCount}");
            }
        }
        m_Report["stopped_by"] = File.Exists(stopFile) ? "stop_file" : "max_seconds";
        m_Report["duration_s"] = Time.realtimeSinceStartupAsDouble - t0;
        m_Report["publish_hz_samples_10s"] = hzSamples;
        m_Report["result"] = "RAN";
    }

    [TearDown]
    public void TearDown()
    {
        JointStateMirror.ForcedMode = null;
        if (m_OutDir == null || m_Mirror == null) return;
        m_Report["connected_at_end"] = m_Mirror.Connected;
        m_Report["commands_received"] = m_Mirror.MessageCount;
        m_Report["unknown_names"] = m_Mirror.UnknownNames.ToArray();
        m_Report["state_published"] = m_Pub != null ? m_Pub.PublishCount : -1;
        m_Report["berries_published"] = m_Berry != null ? m_Berry.PublishCount : -1;
        m_Report["berries_last_published_count"] = m_Berry != null ? m_Berry.LastPublishedCount : -1;
        var follower = UnityEngine.Object.FindObjectOfType<GraspFollower>();
        if (follower != null)
        {
            m_Report["grasp_attach"] = follower.AttachCount;
            m_Report["grasp_release"] = follower.ReleaseCount;
            m_Report["grasp_ignored"] = follower.IgnoredCount;
            m_Report["grasp_placed_in_tray"] = follower.PlacedCount;
            m_Report["grasp_dropped_outside"] = follower.DroppedCount;
            m_Report["grasp_fruits"] = follower.Fruits.Select(f => $"{f.name} {(f.harvested ? f.outcome : f == follower.Attached ? "attached" : "on_vine")}").ToArray();
            m_Report["grasp_events"] = follower.EventLog.ToArray();
        }
        m_Report["violations"] = m_Monitor.ViolationCount;
        m_Report["near_limit_episodes"] = m_Monitor.NearCount;
        m_Report["operational_warn"] = m_Monitor.OperationalWarnCount;
        m_Report["monitor_value_source"] = m_Monitor.ValueSource;
        m_Report["wrap_detected"] = m_Monitor.WrapDetected;
        Directory.CreateDirectory(m_OutDir);
        File.WriteAllText(Path.Combine(m_OutDir, "unity_plant_report.json"), TestJson.Write(m_Report));
        // 모니터 CSV 는 Play 마다 덮어쓰므로 이 런 폴더로 복사한다. 파일 핸들은 OnDestroy 에서 닫히므로 내용을 읽어 쓴다.
        foreach (var (src, dst) in new[] { (m_Monitor.ViolationsCsvPath, "limit_violations.csv"), (m_Monitor.NearCsvPath, "limit_near.csv") })
        {
            try
            {
                using (var fs = new FileStream(src, FileMode.Open, FileAccess.Read, FileShare.ReadWrite))
                using (var sr = new StreamReader(fs)) File.WriteAllText(Path.Combine(m_OutDir, dst), sr.ReadToEnd());
            }
            catch (Exception e) { Debug.LogWarning($"[M2PlantRun] {dst} 복사 실패: {e.Message}"); }
        }
        Debug.Log("[M2PlantRun] 보고서 → " + Path.Combine(m_OutDir, "unity_plant_report.json"));
    }
}
