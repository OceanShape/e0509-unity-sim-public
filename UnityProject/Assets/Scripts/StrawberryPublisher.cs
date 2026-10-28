// 단계 7 — M2 과실 좌표: /isaac_sim/strawberries 발행 (CONTRACT §3-1·§3-6, D-6 — 10-21 개정).
//
// geometry_msgs/PoseArray, frame_id "world", 벽시계 약 1 Hz, orientation 항등, 위치 = 익은 과실 중심(m, ROS Z-up).
// 좌표는 CONTRACT §3-1 고정 목록이고, Isaac 처럼 그리퍼에 붙은 과실과 수확된(RELEASE 된) 과실은 뺀다(GraspFollower.IsExcluded).
// 10-21 전까지는 정적 목록 그대로 냈다(D-6 첫 결정). 그때는 수확된 과실이 플래너의 이웃 장애물로 남았다.
// 안 익은 과실은 발행하지 않는다(내면 플래너가 수확을 시도해 원본보다 관대해진다, CONTRACT §3-1).
// 화면에는 SimSetup 이 Isaac 딸기 메쉬를 놓는다(Environment/Strawberries). 그 전에는 자리표시 구(빨강·초록)를 만든다. 콜라이더 없음.
// Mirror 모드(M1)에서는 발행하지 않는다(bag 재생이 같은 토픽을 낼 수 있다). 구는 두 모드 모두 보인다.
using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

public class StrawberryPublisher : MonoBehaviour
{
    public JointStateMirror mirror;
    [Tooltip("붙은·수확된 과실을 발행에서 뺀다. 비우면 씬에서 찾는다")]
    public GraspFollower follower;
    public string topic = "/isaac_sim/strawberries";
    public float periodSeconds = 1.0f;          // Isaac: 벽시계 1.0 s 게이트
    public float berryRadius = 0.017f;          // 화면용 (지름 34 mm)
    [Tooltip("Isaac 딸기 메쉬(Environment/Strawberries)가 있으면 SimSetup 이 끈다. 켜져 있으면 자리표시 구를 만든다")]
    public bool buildPlaceholderSpheres = true;

    // CONTRACT §3-1 시연 배치 (ROS world, m). 출처 layout_layer.usd:228-321
    public Vector3[] ripeRos =
    {
        new Vector3(-0.25f, 0.7828f, 0.74f), new Vector3(-0.12f, 0.7828f, 0.76f),
        new Vector3( 0.20f, 0.7828f, 0.74f), new Vector3(-0.22f, 0.7828f, 0.56f),
        new Vector3(-0.10f, 0.7828f, 0.44f), new Vector3(-0.25f, 0.7828f, 0.50f),
        new Vector3(-0.38f, 0.7828f, 0.72f), new Vector3( 0.28f, 0.7828f, 0.70f),
    };
    public Vector3[] unripeRos =
    {
        new Vector3(0.15f, 0.7828f, 0.90f), new Vector3(0.12f, 0.7828f, 0.56f),
        new Vector3(0.25f, 0.7828f, 0.48f), new Vector3(0.18f, 0.7828f, 0.38f),
    };

    public long PublishCount { get; private set; }
    public bool Active { get; private set; }
    /// 마지막으로 낸 과실 수 (붙은·수확된 과실을 뺀 뒤)
    public int LastPublishedCount { get; private set; } = -1;

    ROSConnection m_Ros;
    double m_NextRealtime;

    // URDF-Importer 와 같은 변환: ROS (x, y, z) → Unity (−y, z, x). 로봇 베이스 = 월드 원점(CONTRACT §3-6)
    public static Vector3 RosToUnity(Vector3 r) => new Vector3(-r.y, r.z, r.x);

    void Start()
    {
        if (buildPlaceholderSpheres) BuildVisuals();
        if (mirror == null) mirror = GetComponent<JointStateMirror>();
        if (mirror == null) mirror = FindObjectOfType<JointStateMirror>();
        if (follower == null) follower = GetComponent<GraspFollower>();
        if (follower == null) follower = FindObjectOfType<GraspFollower>();
        if (mirror == null || mirror.mode != JointStateMirror.ApplyMode.Command)
        {
            Debug.Log($"[StrawberryPublisher] mode={(mirror == null ? "?" : mirror.mode.ToString())} — 발행 안 함 (Command 모드에서만). 구만 표시");
            return;
        }
        if (!JointStateMirror.SkipRos)
        {
            m_Ros = ROSConnection.GetOrCreateInstance();
            m_Ros.RegisterPublisher<PoseArrayMsg>(topic, 10);
        }
        Active = true;
        Debug.Log($"[StrawberryPublisher] {topic} 발행 시작: 익은 과실 {ripeRos.Length}개(붙은·수확된 과실 뺌{(follower == null ? " — GraspFollower 없음, 정적 목록" : "")}), {periodSeconds:F1} s 주기, frame 'world'");
    }

    void BuildVisuals()
    {
        var root = transform.Find("Strawberries");
        if (root != null) return;                      // 씬에 이미 있음(BuildScene 또는 이전 Play)
        root = new GameObject("Strawberries").transform;
        root.SetParent(transform, false);
        var shader = Shader.Find("Standard");
        Material ripe = shader != null ? new Material(shader) { color = new Color(0.85f, 0.08f, 0.1f) } : null;
        Material unripe = shader != null ? new Material(shader) { color = new Color(0.45f, 0.75f, 0.25f) } : null;
        void Make(string name, Vector3 ros, Material m)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            go.name = name;
            var col = go.GetComponent<Collider>();
            if (col != null) Destroy(col);
            go.transform.SetParent(root, false);
            go.transform.position = RosToUnity(ros);
            go.transform.localScale = Vector3.one * berryRadius * 2f;
            if (m != null) go.GetComponent<Renderer>().sharedMaterial = m;
        }
        for (int i = 0; i < ripeRos.Length; i++) Make($"ripe_{i + 1:00}", ripeRos[i], ripe);
        for (int i = 0; i < unripeRos.Length; i++) Make($"unripe_{i + 1:00}", unripeRos[i], unripe);
    }

    void Update()
    {
        if (!Active) return;
        double now = Time.realtimeSinceStartupAsDouble;
        if (now < m_NextRealtime) return;
        m_NextRealtime = now + periodSeconds;

        var list = new System.Collections.Generic.List<PoseMsg>(ripeRos.Length);
        for (int i = 0; i < ripeRos.Length; i++)
            if (follower == null || !follower.IsExcluded(i))
                list.Add(new PoseMsg(new PointMsg(ripeRos[i].x, ripeRos[i].y, ripeRos[i].z), new QuaternionMsg()));
        if (list.Count != LastPublishedCount)
            Debug.Log($"[StrawberryPublisher] 발행 과실 {LastPublishedCount} → {list.Count}개");
        LastPublishedCount = list.Count;
        if (m_Ros != null)
        {
            var poses = list.ToArray();
            long ns = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() * 1000000L;
            var header = new HeaderMsg(new TimeMsg((int)(ns / 1000000000L), (uint)(ns % 1000000000L)), "world");
            m_Ros.Publish(topic, new PoseArrayMsg(header, poses));
        }
        PublishCount++;
    }
}
