// 단계 8 — 잡은 과실이 그리퍼를 따라간다: /sim/grasp_event (std_msgs/String) 처리 (CONTRACT §3-1·D-6, 10-21 개정).
//
// Isaac 브릿지(isaac_sim_script_editor_bridge.py _attach·_release·_follow_attached·_in_tray)와 같은 판정이다.
//   "ATTACH x y z"  아직 수확되지 않은 익은 과실 가운데 (x, y, z)(ROS world, m)에 가장 가까운 것. 60 mm 보다 멀면 무시,
//                   이미 하나가 붙어 있으면 무시. 붙는 순간의 그리퍼 기준 상대 자세를 지킨 채 rh_p12_rn_base 를 따라간다
//                   (Isaac 은 T_rel 을 매 스텝 다시 쓰고, Unity 는 링크의 자식으로 옮긴다 — 결과는 같다).
//   "RELEASE"       떼어 내고 수확됨으로 표시. 계란판 상자(수평 ±30 mm, 윗면 위 0.30 m 이하) 안이면 트레이 배치, 밖이면 낙하.
//                   둘 다 놓인 자리에서 아래로 떨어뜨린다. Isaac 은 동적 강체로 바꿔 PhysX 로 떨어뜨리지만, 여기서는 **운동학적 낙하**다:
//                   과실 정점마다 아래 방향으로 계란판·책상·상자 면을 찾아 처음 닿는 높이까지 중력 가속으로 내린다(굴러가지 않는다).
//                   콜라이더를 더하지 않는 이유: URDF 임포트가 손가락 링크에 콜라이더를 만들어, 강체 과실이 벌린 손가락 사이에서 튕길 수
//                   있다(Isaac 은 손끝 콜라이더가 없다). 환경에 콜라이더가 없으니 로봇 동작에도 영향이 없다.
//   붙은 과실·수확된 과실은 /isaac_sim/strawberries 에서 뺀다(StrawberryPublisher 가 IsExcluded 를 묻는다) — Isaac 과 같다.
// 두 모드 모두 돈다: Mirror(M1)는 bag 에 담긴 /sim/grasp_event, Command(M2)는 sim_executor_bridge 가 내는 것을 받는다.
// Mirror 에서 bag 반복 재생이 처음으로 돌아가면(관절 메시지 stamp 가 1 s 넘게 뒤로 감) 과실을 처음 자리로 되돌린다.
using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.Std;

public class GraspFollower : MonoBehaviour
{
    public JointStateMirror mirror;
    public StrawberryPublisher berries;
    public string topic = "/sim/grasp_event";
    public string gripperLinkName = "rh_p12_rn_base";      // Isaac GRIPPER_BASE_PRIM (cuRobo ee_link)
    public float attachMatchMax = 0.06f;                    // Isaac ATTACH_MATCH_MAX_M
    public float trayMargin = 0.03f;                        // Isaac TRAY_XY_MARGIN_M (수평 두 축)
    public float trayAboveMax = 0.30f;                      // Isaac TRAY_Z_ABOVE_M
    public float floorTopY = -0.45f;                        // Isaac FLOOR_TOP_M — 아래에 면이 없을 때 멈추는 높이
    public string trayName = "egg_carton";

    public class Fruit
    {
        public string name;
        public int index;                 // StrawberryPublisher.ripeRos 순서 (발행 순서)
        public Transform t;               // 화면 과실(없을 수 있다)
        public Vector3 rootUnity;         // 과실 루트 좌표 = CONTRACT §3-1 (Isaac 루트 프림 위치), Unity world
        public Transform homeParent;
        public Vector3 homeLocalPos;
        public Quaternion homeLocalRot;
        public bool harvested;
        public string outcome = "";       // "" | "placed" | "dropped"
        public Coroutine drop;
    }

    public IReadOnlyList<Fruit> Fruits => m_Fruits;
    public Fruit Attached { get; private set; }
    public int AttachCount { get; private set; }
    public int ReleaseCount { get; private set; }
    public int IgnoredCount { get; private set; }
    public int PlacedCount { get; private set; }
    public int DroppedCount { get; private set; }
    public int ResetCount { get; private set; }
    public int UnripeCount { get; private set; }
    public Bounds TrayBounds => m_Tray;
    public readonly List<string> EventLog = new List<string>();

    readonly List<Fruit> m_Fruits = new List<Fruit>();
    Transform m_Gripper;
    Bounds m_Tray;
    bool m_HaveTray;
    double m_LastStamp = double.NaN;
    DropSurface m_Surface;

    public static Vector3 RosToUnity(Vector3 r) => StrawberryPublisher.RosToUnity(r);
    public static Vector3 UnityToRos(Vector3 u) => new Vector3(u.z, -u.x, u.y);

    void Start()
    {
        if (mirror == null) mirror = GetComponent<JointStateMirror>();
        if (mirror == null) mirror = FindObjectOfType<JointStateMirror>();
        if (berries == null) berries = GetComponent<StrawberryPublisher>();
        if (berries == null) berries = FindObjectOfType<StrawberryPublisher>();
        var robot = FindObjectOfType<UrdfRobot>();
        m_Gripper = robot != null ? FindDeep(robot.transform, gripperLinkName) : null;
        if (m_Gripper == null) { Debug.LogError($"[GraspFollower] 링크 '{gripperLinkName}' 없음 — 비활성"); enabled = false; return; }
        BuildFruitList();
        var tray = GameObject.Find(trayName);
        var tr = tray != null ? tray.GetComponent<Renderer>() : null;
        if (tr != null) { m_Tray = tr.bounds; m_HaveTray = true; }
        else Debug.LogWarning($"[GraspFollower] 트레이 '{trayName}' 없음 — RELEASE 는 모두 트레이 밖으로 센다 (Isaac 과 같은 처리)");
        if (m_HaveTray)
            Debug.Log($"[GraspFollower] 트레이 상자 x[{m_Tray.min.x - trayMargin:F3}, {m_Tray.max.x + trayMargin:F3}] z[{m_Tray.min.z - trayMargin:F3}, {m_Tray.max.z + trayMargin:F3}] y≤{m_Tray.max.y + trayAboveMax:F3} (Unity world, {trayName} 경계 + 여유)");
        if (!JointStateMirror.SkipRos)
        {
            ROSConnection.GetOrCreateInstance().Subscribe<StringMsg>(topic, m => Handle(m.data));
            Debug.Log($"[GraspFollower] 구독 {topic} (mode={(mirror != null ? mirror.mode.ToString() : "?")})");
        }
    }

    // 과실 목록: 발행 좌표(ripeRos)마다 화면 과실 하나를 가장 가까운 것으로 짝짓는다. 이름 규칙은 Isaac 필터와 같다(ripe 포함, unripe 아님).
    void BuildFruitList()
    {
        Transform root = null;
        var env = GameObject.Find("Environment");
        if (env != null) root = env.transform.Find("Strawberries");
        if (root == null && berries != null) root = berries.transform.Find("Strawberries");   // 자리표시 구
        var candidates = new List<Transform>();
        if (root != null)
            foreach (Transform c in root)
            {
                string n = c.name.ToLowerInvariant();
                if (n.Contains("unripe")) UnripeCount++;
                else if (n.Contains("ripe")) candidates.Add(c);
            }
        var ripe = berries != null ? berries.ripeRos : new Vector3[0];
        var lines = new List<string>();
        for (int i = 0; i < ripe.Length; i++)
        {
            var f = new Fruit { index = i, rootUnity = RosToUnity(ripe[i]), name = $"ripe[{i}]" };
            Transform best = null; float bestD = float.MaxValue;
            foreach (var c in candidates)
            {
                float d = Vector3.Distance(Center(c), f.rootUnity);
                if (d < bestD) { bestD = d; best = c; }
            }
            if (best != null)
            {
                candidates.Remove(best);
                f.t = best; f.name = best.name;
                f.homeParent = best.parent; f.homeLocalPos = best.localPosition; f.homeLocalRot = best.localRotation;
                lines.Add($"{best.name}↔({ripe[i].x:F2},{ripe[i].y:F4},{ripe[i].z:F2}) {bestD * 1000f:F1}mm");
                if (bestD > 0.02f) Debug.LogWarning($"[GraspFollower] {best.name} 메쉬 중심이 발행 좌표에서 {bestD * 1000f:F1} mm — 짝이 틀렸을 수 있다");
            }
            m_Fruits.Add(f);
        }
        Debug.Log($"[GraspFollower] 익은 과실 {m_Fruits.Count}개(화면 {m_Fruits.Count(f => f.t != null)}), 안 익은 {UnripeCount}개: " + string.Join(", ", lines));
    }

    static Vector3 Center(Transform t)
    {
        var r = t.GetComponent<Renderer>();
        return r != null ? r.bounds.center : t.position;
    }

    /// 발행에서 뺄 과실인가 (붙어 있거나 수확됨) — StrawberryPublisher 가 묻는다
    public bool IsExcluded(int index)
    {
        if (index < 0 || index >= m_Fruits.Count) return false;
        var f = m_Fruits[index];
        return f.harvested || f == Attached;
    }

    void Update()
    {
        // bag 반복 재생이 처음으로 돌아가면 받은 관절 메시지의 stamp 가 뒤로 간다
        if (mirror != null && mirror.mode == JointStateMirror.ApplyMode.Mirror && mirror.LastMessage != null && mirror.LastMessage.header != null)
        {
            var st = mirror.LastMessage.header.stamp;
            double s = st.sec + st.nanosec * 1e-9;
            if (s > 0)
            {
                if (!double.IsNaN(m_LastStamp) && s < m_LastStamp - 1.0) ResetAll($"관절 메시지 stamp {m_LastStamp:F1} → {s:F1} s (bag 처음으로)");
                m_LastStamp = s;
            }
        }
    }

    /// ROS 콜백과 같은 경로 (테스트용으로도 쓴다)
    public void Handle(string ev)
    {
        var parts = (ev ?? "").Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length == 4 && parts[0] == "ATTACH"
            && float.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out var x)
            && float.TryParse(parts[2], NumberStyles.Float, CultureInfo.InvariantCulture, out var y)
            && float.TryParse(parts[3], NumberStyles.Float, CultureInfo.InvariantCulture, out var z))
            Attach(new Vector3(x, y, z));
        else if (parts.Length >= 1 && parts[0] == "RELEASE")
            Release();
        else
            Note($"모르는 grasp 이벤트: '{ev}'", ignored: true);
    }

    void Attach(Vector3 ros)
    {
        if (Attached != null) { Note($"ATTACH 무시: 이미 {Attached.name} 이(가) 붙어 있다", ignored: true); return; }
        Vector3 target = RosToUnity(ros);
        Fruit best = null; float bestD = float.MaxValue;
        foreach (var f in m_Fruits)
        {
            if (f.harvested) continue;
            float d = Vector3.Distance(f.rootUnity, target);
            if (d < bestD) { bestD = d; best = f; }
        }
        if (best == null || bestD > attachMatchMax)
        {
            Note($"ATTACH 무시: ({ros.x:F3}, {ros.y:F3}, {ros.z:F3}) 에서 {attachMatchMax * 1000f:F0} mm 안에 익은 과실 없음 (가장 가까운 {(best == null ? "없음" : (bestD * 1000f).ToString("F0") + " mm")})", ignored: true);
            return;
        }
        Attached = best;
        AttachCount++;
        string off = "";
        if (best.t != null)
        {
            if (best.drop != null) { StopCoroutine(best.drop); best.drop = null; }
            best.t.SetParent(m_Gripper, worldPositionStays: true);
            // ROS 링크 기준 오프셋 (Isaac 로그와 같은 축으로 보이게)
            Vector3 l = UnityToRos(m_Gripper.InverseTransformPoint(Center(best.t))) * 1000f;
            off = $"  그리퍼 기준 ({l.x:F1}, {l.y:F1}, {l.z:F1}) mm";
        }
        Note($"ATTACH {best.name}  일치 {bestD * 1000f:F1} mm{off}");
    }

    void Release()
    {
        if (Attached == null) { Note("RELEASE 무시: 붙은 과실 없음", ignored: true); return; }
        var f = Attached;
        Attached = null;
        f.harvested = true;
        ReleaseCount++;
        Vector3 pos = f.t != null ? Center(f.t) : f.rootUnity;
        bool inTray = InTray(pos);
        f.outcome = inTray ? "placed" : "dropped";
        if (inTray) PlacedCount++; else DroppedCount++;
        if (f.t != null)
        {
            f.t.SetParent(f.homeParent, worldPositionStays: true);
            f.drop = StartCoroutine(Drop(f, inTray));
        }
        Vector3 r = UnityToRos(pos) * 1000f;
        int harvested = m_Fruits.Count(x => x.harvested);
        Note(inTray
            ? $"RELEASE {f.name}  트레이 배치, ({r.x:F1}, {r.y:F1}, {r.z:F1}) mm 에서 놓음 → 계란판으로 떨어짐  수확 {harvested}"
            : $"RELEASE {f.name}  트레이 밖 낙하, ({r.x:F1}, {r.y:F1}, {r.z:F1}) mm 에서 놓음  수확 {harvested} 낙하 {DroppedCount}");
    }

    public bool InTray(Vector3 p)
    {
        if (!m_HaveTray) return false;
        return p.x >= m_Tray.min.x - trayMargin && p.x <= m_Tray.max.x + trayMargin
            && p.z >= m_Tray.min.z - trayMargin && p.z <= m_Tray.max.z + trayMargin
            && p.y <= m_Tray.max.y + trayAboveMax;
    }

    IEnumerator Drop(Fruit f, bool inTray)
    {
        if (m_Surface == null) m_Surface = DropSurface.FromEnvironment();
        float dist = m_Surface.DropDistance(f.t, floorTopY);
        Vector3 p0 = f.t.position;
        float t = 0f, fallen = 0f;
        while (fallen < dist)
        {
            t += Time.deltaTime;
            fallen = Mathf.Min(dist, 0.5f * 9.81f * t * t);
            f.t.position = p0 - Vector3.up * fallen;
            yield return null;
        }
        f.drop = null;
        Vector3 r = UnityToRos(Center(f.t)) * 1000f;
        bool restIn = InTray(Center(f.t));
        Debug.Log($"[GraspFollower] {f.name} 멈춤 ({r.x:F1}, {r.y:F1}, {r.z:F1}) mm, {dist * 1000f:F1} mm 낙하, {(restIn ? "트레이 안" : "트레이 밖")}{(inTray && !restIn ? " (!! 트레이에 놓았는데 밖)" : "")}");
    }

    public void ResetAll(string why)
    {
        foreach (var f in m_Fruits)
        {
            if (f.drop != null) { StopCoroutine(f.drop); f.drop = null; }
            if (f.t != null) { f.t.SetParent(f.homeParent, false); f.t.localPosition = f.homeLocalPos; f.t.localRotation = f.homeLocalRot; }
            f.harvested = false; f.outcome = "";
        }
        Attached = null;
        ResetCount++;
        Note($"과실을 처음 자리로: {why}");
    }

    void Note(string msg, bool ignored = false)
    {
        if (ignored) IgnoredCount++;
        EventLog.Add($"{Time.timeAsDouble:F2}s {msg}");
        Debug.Log("[GraspFollower] " + msg);
    }

    static Transform FindDeep(Transform root, string name)
    {
        if (root.name == name) return root;
        foreach (Transform c in root) { var r = FindDeep(c, name); if (r != null) return r; }
        return null;
    }

    // ── 운동학적 낙하용 받침면: 계란판·책상 상판 삼각형(수평 격자에 담음) + 상자 윗면(바닥·다리·받침판) ──
    class DropSurface
    {
        const float k_Cell = 0.01f;
        Vector3[] m_Tri = new Vector3[0];
        readonly Dictionary<long, List<int>> m_Grid = new Dictionary<long, List<int>>();
        readonly List<Bounds> m_Boxes = new List<Bounds>();

        public static DropSurface FromEnvironment()
        {
            var s = new DropSurface();
            var env = GameObject.Find("Environment");
            if (env == null) { Debug.LogWarning("[GraspFollower] Environment 없음 — 낙하는 바닥 높이까지"); return s; }
            var tris = new List<Vector3>();
            foreach (var mf in env.GetComponentsInChildren<MeshFilter>())
            {
                string n = mf.name;
                bool isBox = mf.transform.parent != null && mf.transform.parent.name == "Boxes";
                if (isBox) { var r = mf.GetComponent<Renderer>(); if (r != null) s.m_Boxes.Add(r.bounds); continue; }
                if (n != "egg_carton" && n != "table_top") continue;
                var m = mf.sharedMesh; var M = mf.transform.localToWorldMatrix;
                var v = m.vertices; var idx = m.triangles;
                for (int i = 0; i < idx.Length; i++) tris.Add(M.MultiplyPoint3x4(v[idx[i]]));
            }
            s.m_Tri = tris.ToArray();
            for (int t = 0; t < s.m_Tri.Length / 3; t++)
            {
                Vector3 a = s.m_Tri[3 * t], b = s.m_Tri[3 * t + 1], c = s.m_Tri[3 * t + 2];
                if (Mathf.Abs((b.x - a.x) * (c.z - a.z) - (c.x - a.x) * (b.z - a.z)) < 1e-10f) continue;   // 수직 면은 받침이 아니다
                int x0 = Cell(Mathf.Min(a.x, b.x, c.x)), x1 = Cell(Mathf.Max(a.x, b.x, c.x));
                int z0 = Cell(Mathf.Min(a.z, b.z, c.z)), z1 = Cell(Mathf.Max(a.z, b.z, c.z));
                for (int x = x0; x <= x1; x++) for (int z = z0; z <= z1; z++)
                {
                    long k = Key(x, z);
                    if (!s.m_Grid.TryGetValue(k, out var l)) s.m_Grid[k] = l = new List<int>();
                    l.Add(t);
                }
            }
            Debug.Log($"[GraspFollower] 낙하 받침면: 삼각형 {s.m_Tri.Length / 3}, 상자 {s.m_Boxes.Count}");
            return s;
        }

        static int Cell(float v) => Mathf.FloorToInt(v / k_Cell);
        static long Key(int x, int z) => ((long)x << 32) ^ (uint)z;

        // p 바로 아래(또는 같은 높이)의 가장 높은 받침면 높이. 없으면 −∞
        float SurfaceBelow(Vector3 p)
        {
            float best = float.NegativeInfinity;
            if (m_Grid.TryGetValue(Key(Cell(p.x), Cell(p.z)), out var l))
                foreach (int t in l)
                {
                    Vector3 a = m_Tri[3 * t], b = m_Tri[3 * t + 1], c = m_Tri[3 * t + 2];
                    float d = (b.x - a.x) * (c.z - a.z) - (c.x - a.x) * (b.z - a.z);
                    float u = ((p.x - a.x) * (c.z - a.z) - (c.x - a.x) * (p.z - a.z)) / d;
                    float w = ((b.x - a.x) * (p.z - a.z) - (p.x - a.x) * (b.z - a.z)) / d;
                    if (u < 0f || w < 0f || u + w > 1f) continue;
                    float y = a.y + u * (b.y - a.y) + w * (c.y - a.y);
                    if (y <= p.y + 1e-4f && y > best) best = y;
                }
            foreach (var bx in m_Boxes)
                if (p.x >= bx.min.x && p.x <= bx.max.x && p.z >= bx.min.z && p.z <= bx.max.z && bx.max.y <= p.y + 1e-4f && bx.max.y > best)
                    best = bx.max.y;
            return best;
        }

        // 과실을 똑바로 내릴 때 처음 닿기까지의 거리: 정점마다 아래 받침면까지의 틈 중 최소
        public float DropDistance(Transform fruit, float floorY)
        {
            var mf = fruit.GetComponent<MeshFilter>();
            if (mf == null || mf.sharedMesh == null) return Mathf.Max(0f, fruit.position.y - floorY);
            var M = fruit.localToWorldMatrix;
            float gap = float.PositiveInfinity, lowest = float.PositiveInfinity;
            foreach (var v in mf.sharedMesh.vertices)
            {
                Vector3 w = M.MultiplyPoint3x4(v);
                lowest = Mathf.Min(lowest, w.y);
                float s = SurfaceBelow(w);
                if (!float.IsNegativeInfinity(s)) gap = Mathf.Min(gap, w.y - s);
            }
            if (float.IsPositiveInfinity(gap)) gap = lowest - floorY;
            return Mathf.Max(0f, gap);
        }
    }
}
