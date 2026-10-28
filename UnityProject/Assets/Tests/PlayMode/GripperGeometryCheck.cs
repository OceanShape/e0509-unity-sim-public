// 그리퍼 형상 검사: 같은 관절각에서 Unity 의 좌우 커스텀 손가락 사이 최소 거리를 URDF 순기구학(trimesh) 값과 비교한다.
// URDF 기준(scripts 로 계산): 53.05° 17.82 mm · 57.30° 9.40 mm · 61.88°(가상 제어기 닫힘 1.08 rad) 0.29 mm · 63.03°(상한) 겹침.
// 결과: logs/m2/gripper_geometry.json. 단언: 61.88° 까지 겹치지 않고 URDF 값과 ±2 mm.
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using RosMessageTypes.Sensor;

public class GripperGeometryCheck
{
    static readonly (float deg, float urdfMm)[] k_Cases = { (0f, 102.76f), (30f, 61.03f), (53.05f, 17.82f), (57.3f, 9.40f), (61.88f, 0.29f) };
    static readonly string[] k_Names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "rh_p12_rn", "rh_r2", "rh_l1", "rh_l2" };
    static readonly double[] k_Arm = { 87.98, -94.92, 129.89, 175.94, -31.34, 93.42 };

    static Vector3[] WorldVerts(Transform part)
    {
        var list = new List<Vector3>();
        foreach (var mf in part.GetComponentsInChildren<MeshFilter>(true))
        {
            if (mf.sharedMesh == null) continue;
            var m = mf.transform.localToWorldMatrix;
            foreach (var v in mf.sharedMesh.vertices) list.Add(m.MultiplyPoint3x4(v));
        }
        return list.ToArray();
    }

    static float MinDist(Vector3[] a, Vector3[] b)
    {
        // 격자 해시로 근사 최근접 (정점 수천 개)
        float best = float.MaxValue;
        float cell = 0.01f;
        var grid = new Dictionary<Vector3Int, List<Vector3>>();
        foreach (var p in b)
        {
            var k = Vector3Int.FloorToInt(p / cell);
            if (!grid.TryGetValue(k, out var l)) grid[k] = l = new List<Vector3>();
            l.Add(p);
        }
        foreach (var p in a)
        {
            var k = Vector3Int.FloorToInt(p / cell);
            for (int dx = -1; dx <= 1; dx++) for (int dy = -1; dy <= 1; dy++) for (int dz = -1; dz <= 1; dz++)
                if (grid.TryGetValue(k + new Vector3Int(dx, dy, dz), out var l))
                    foreach (var q in l) best = Mathf.Min(best, (p - q).sqrMagnitude);
        }
        if (best == float.MaxValue)   // 1 cm 안에 없으면 전수
            foreach (var p in a) foreach (var q in b) best = Mathf.Min(best, (p - q).sqrMagnitude);
        return Mathf.Sqrt(best);
    }

    [UnityTest]
    public IEnumerator CustomFingers_MatchUrdfGap()
    {
        JointStateMirror.ForcedMode = JointStateMirror.ApplyMode.Mirror;
        JointStateMirror.SkipRos = true;
        yield return SceneManager.LoadSceneAsync("Assets/Scenes/E0509Sim.unity", LoadSceneMode.Single);
        yield return null;
        var m = Object.FindObjectOfType<JointStateMirror>();
        var left = GameObject.Find("left_custom_part").transform;
        var right = GameObject.Find("right_custom_part").transform;
        var rows = new List<string>();
        var fails = new List<string>();
        foreach (var (deg, urdf) in k_Cases)
        {
            var p = new double[10];
            for (int i = 0; i < 6; i++) p[i] = k_Arm[i] * Mathf.Deg2Rad;
            for (int i = 6; i < 10; i++) p[i] = deg * Mathf.Deg2Rad;
            m.InjectMessage(new JointStateMsg { name = (string[])k_Names.Clone(), position = p });
            for (int s = 0; s < 5; s++) yield return new WaitForFixedUpdate();
            float d = MinDist(WorldVerts(left), WorldVerts(right)) * 1000f;
            var meas = string.Join(",", new[] { "rh_p12_rn", "rh_r2", "rh_l1", "rh_l2" }.Select(j => (m.Joints[j].jointPosition[0] * Mathf.Rad2Deg).ToString("F2")));
            rows.Add($"{{\"deg\":{deg},\"unity_mm\":{d:F2},\"urdf_mm\":{urdf},\"joints_deg\":\"{meas}\"}}");
            Debug.Log($"[GripperGeometryCheck] {deg,6:F2}°  Unity {d,7:F2} mm  URDF {urdf,7:F2} mm  관절 [{meas}]");
            if (Mathf.Abs(d - urdf) > 2f) fails.Add($"{deg}°: Unity {d:F2} vs URDF {urdf:F2} mm");
        }
        string dir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "logs", "m2"));
        Directory.CreateDirectory(dir);
        File.WriteAllText(Path.Combine(dir, "gripper_geometry.json"), "[" + string.Join(",", rows) + "]");
        Assert.IsEmpty(fails, "Unity 손가락 간격이 URDF 와 다르다: " + string.Join("; ", fails));
    }

    [TearDown]
    public void TearDown() { JointStateMirror.ForcedMode = null; JointStateMirror.SkipRos = false; }
}
