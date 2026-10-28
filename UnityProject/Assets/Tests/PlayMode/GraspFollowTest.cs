// 잡은 과실 따라가기 시험 (단계 8, GraspFollower). ROS 없이(SkipRos) Isaac 런 bag 의 실제 관절값과 grasp 이벤트를 그대로 넣는다.
//   ~/bags/run1 의 첫 픽: 69.21 s "ATTACH -0.3800 0.7828 0.7200" → 77.75 s "RELEASE" (관절값은 bag /dsr01/joint_states 에서 뽑은 것)
// 확인:
//   1. ATTACH 순간 TCP 가 그 과실 가까이 있다(bag 자세 → Unity 순기구학 → 과실 좌표가 한 사슬로 맞는다)
//   2. 붙은 뒤 팔이 움직여도 그리퍼 기준 상대 위치가 그대로다(0.1 mm), 과실은 처음 자리에서 멀어진다
//   3. 붙은·수확된 과실은 발행 제외(IsExcluded), Isaac 과 같은 무시 규칙(이미 붙음 · 60 mm 밖 · 붙은 것 없이 RELEASE)
//   4. RELEASE 뒤 과실이 아래로 떨어져 멈춘다. 계란판 위에서 놓으면 계란판 면(컵·턱)에 닿아 상자 경계 안에서 멈춘다
// 결과: logs/m2/grasp_follow_test.json
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using RosMessageTypes.Sensor;

public class GraspFollowTest
{
    static readonly string[] k_Names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "rh_l1", "rh_p12_rn", "rh_l2", "rh_r2" };
    // bag 시각(s) → 관절값(deg, 위 순서)
    static readonly (float t, double[] deg)[] k_Bag =
    {
        (69.21f, new[] { 136.582, 58.684, -26.539, 126.572, -64.061, 115.813, 53.04, 53.039, 53.04, 53.039 }),
        (71.00f, new[] { 136.578, 62.851, -32.985, 127.455, -65.473, 113.741, 61.879, 61.879, 61.879, 61.879 }),
        (73.00f, new[] { 136.579, 68.334, -41.070, 128.353, -67.063, 111.511, 61.88, 61.88, 61.879, 61.879 }),
        (75.00f, new[] { 138.628, 70.313, -46.610, 127.407, -69.973, 109.034, 61.879, 61.879, 61.879, 61.879 }),
        (77.75f, new[] { 139.947, 71.336, -49.650, 126.642, -71.590, 107.659, 61.879, 61.879, 61.879, 61.879 }),
    };

    JointStateMirror m_Mirror;

    IEnumerator Pose(double[] deg)
    {
        m_Mirror.InjectMessage(new JointStateMsg { name = (string[])k_Names.Clone(), position = deg.Select(d => d * Mathf.Deg2Rad).ToArray() });
        for (int i = 0; i < 6; i++) yield return new WaitForFixedUpdate();
    }

    static float LowestY(Transform t)
    {
        var mf = t.GetComponent<MeshFilter>(); var M = t.localToWorldMatrix;
        return mf.sharedMesh.vertices.Min(v => M.MultiplyPoint3x4(v).y);
    }

    [UnityTest]
    public IEnumerator Attach_Follows_Release_Drops()
    {
        JointStateMirror.ForcedMode = JointStateMirror.ApplyMode.Mirror;
        JointStateMirror.SkipRos = true;
        yield return SceneManager.LoadSceneAsync("Assets/Scenes/E0509Sim.unity", LoadSceneMode.Single);
        yield return null;
        m_Mirror = Object.FindObjectOfType<JointStateMirror>();
        var f = Object.FindObjectOfType<GraspFollower>();
        Assert.IsNotNull(f, "GraspFollower 가 씬에 없다");
        Assert.AreEqual(8, f.Fruits.Count, "익은 과실 8개");
        Assert.IsTrue(f.Fruits.All(x => x.t != null), "익은 과실마다 화면 과실이 짝지어져야 한다");
        var grip = GameObject.Find("rh_p12_rn_base").transform;
        var tcp = grip.Find("tcp");
        var rep = new List<string>();

        // 1. ATTACH 순간
        yield return Pose(k_Bag[0].deg);
        f.Handle("ATTACH -0.3800 0.7828 0.7200");
        Assert.IsNotNull(f.Attached, "ATTACH 가 과실을 붙이지 않았다");
        var fr = f.Attached;
        Vector3 home = fr.t.position;
        float tcpToRoot = Vector3.Distance(tcp.position, fr.rootUnity);
        rep.Add($"\"attached\":\"{fr.name}\",\"tcp_to_fruit_root_mm\":{tcpToRoot * 1000f:F1}");
        Debug.Log($"[GraspFollowTest] ATTACH {fr.name}: TCP–과실 루트 {tcpToRoot * 1000f:F1} mm");
        Assert.Less(tcpToRoot, 0.10f, "bag 의 ATTACH 자세에서 TCP 가 과실에서 10 cm 넘게 떨어져 있다(순기구학·좌표 사슬 불일치)");
        Assert.IsTrue(f.IsExcluded(fr.index), "붙은 과실이 발행에서 빠지지 않는다");
        Vector3 rel = grip.InverseTransformPoint(fr.t.position);
        Quaternion relR = Quaternion.Inverse(grip.rotation) * fr.t.rotation;

        // 2. 들고 움직이기
        float maxRelErr = 0f, maxRotErr = 0f;
        for (int i = 1; i < k_Bag.Length; i++)
        {
            yield return Pose(k_Bag[i].deg);
            maxRelErr = Mathf.Max(maxRelErr, Vector3.Distance(grip.InverseTransformPoint(fr.t.position), rel));
            maxRotErr = Mathf.Max(maxRotErr, Quaternion.Angle(Quaternion.Inverse(grip.rotation) * fr.t.rotation, relR));
        }
        float moved = Vector3.Distance(fr.t.position, home);
        rep.Add($"\"carry_rel_err_mm\":{maxRelErr * 1000f:F4},\"carry_rot_err_deg\":{maxRotErr:F4},\"moved_from_home_mm\":{moved * 1000f:F1}");
        Debug.Log($"[GraspFollowTest] 운반: 상대 위치 오차 최대 {maxRelErr * 1000f:F4} mm, 회전 {maxRotErr:F4}°, 처음 자리에서 {moved * 1000f:F1} mm");
        Assert.Less(maxRelErr, 1e-4f, "운반 중 그리퍼 기준 상대 위치가 변했다");
        Assert.Less(maxRotErr, 0.01f, "운반 중 그리퍼 기준 상대 자세가 변했다");
        Assert.Greater(moved, 0.05f, "과실이 그리퍼를 따라 움직이지 않았다");

        // 3. 무시 규칙
        int ign = f.IgnoredCount;
        f.Handle("ATTACH -0.2500 0.7828 0.7400");               // 이미 붙어 있음
        Assert.AreEqual(fr, f.Attached); Assert.AreEqual(ign + 1, f.IgnoredCount);

        // 4. RELEASE (bag 의 놓는 자세) → 낙하
        float relY = fr.t.position.y;
        f.Handle("RELEASE");
        Assert.IsNull(f.Attached); Assert.IsTrue(fr.harvested); Assert.IsTrue(f.IsExcluded(fr.index), "수확된 과실이 발행에서 빠지지 않는다");
        yield return new WaitForSeconds(1.2f);                 // 바닥(−0.45 m)까지 떨어져도 0.6 s
        float y1 = fr.t.position.y;
        yield return new WaitForSeconds(0.3f);
        rep.Add($"\"release_outcome\":\"{fr.outcome}\",\"release_y\":{relY:F4},\"rest_y\":{fr.t.position.y:F4}");
        Debug.Log($"[GraspFollowTest] RELEASE {fr.name}: {fr.outcome}, 높이 {relY:F4} → {fr.t.position.y:F4} m");
        Assert.LessOrEqual(fr.t.position.y, relY + 1e-5f, "놓은 과실이 위로 갔다");
        Assert.AreEqual(y1, fr.t.position.y, 1e-6f, "놓은 과실이 멈추지 않는다");

        f.Handle("RELEASE");                                    // 붙은 것 없음
        f.Handle("ATTACH 0.0 0.0 0.0");                        // 60 mm 안에 없음
        f.Handle("ATTACH -0.3800 0.7828 0.7200");              // 이미 수확된 과실 자리 → 가장 가까운 미수확도 60 mm 밖
        Assert.AreEqual(ign + 4, f.IgnoredCount, "Isaac 과 같은 무시 규칙 3가지");
        Assert.IsNull(f.Attached);

        // 5. 계란판 위에서 놓기: 다른 과실을 붙이고 계란판 중심 30 mm 위로 옮겨 놓는다
        f.Handle("ATTACH 0.2000 0.7828 0.7400");
        var fr2 = f.Attached;
        Assert.IsNotNull(fr2);
        var tb = f.TrayBounds;
        fr2.t.position += new Vector3(tb.center.x, tb.max.y + 0.03f, tb.center.z) - new Vector3(fr2.t.position.x, LowestY(fr2.t), fr2.t.position.z);
        f.Handle("RELEASE");
        Assert.AreEqual("placed", fr2.outcome, "계란판 위에서 놓았는데 트레이 배치로 세지 않았다");
        yield return new WaitForSeconds(0.8f);
        float low = LowestY(fr2.t);
        bool inside = f.InTray(fr2.t.GetComponent<Renderer>().bounds.center);
        rep.Add($"\"tray_drop_lowest_y\":{low:F4},\"tray_top_y\":{tb.max.y:F4},\"tray_bottom_y\":{tb.min.y:F4},\"tray_rest_inside\":{(inside ? "true" : "false")}");
        Debug.Log($"[GraspFollowTest] 계란판 낙하: 과실 가장 낮은 점 {low:F4} m (계란판 {tb.min.y:F4}–{tb.max.y:F4}), 상자 안 {inside}");
        Assert.IsTrue(inside, "계란판에 놓은 과실이 상자 밖에서 멈췄다");
        Assert.LessOrEqual(low, tb.max.y + 0.001f, "계란판 위에 떠 있다(계란판 면에 닿지 않았다)");
        Assert.GreaterOrEqual(low, tb.min.y - 0.002f, "계란판 바닥을 뚫었다");

        // 6. 처음 자리로
        f.ResetAll("시험 끝");
        Assert.IsTrue(f.Fruits.All(x => !x.harvested) && f.Attached == null);
        Assert.Less(Vector3.Distance(fr.t.position, home), 1e-5f, "ResetAll 뒤 처음 자리가 아니다");

        string dir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "logs", "m2"));
        Directory.CreateDirectory(dir);
        File.WriteAllText(Path.Combine(dir, "grasp_follow_test.json"), "{" + string.Join(",", rep) + ",\"events\":[" + string.Join(",", f.EventLog.Select(e => "\"" + e.Replace("\"", "'") + "\"")) + "]}");
    }

    [TearDown]
    public void TearDown() { JointStateMirror.ForcedMode = null; JointStateMirror.SkipRos = false; }
}
