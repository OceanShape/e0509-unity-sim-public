// 단계 5 — TCP 궤적 (CONTRACT §2-6).
//
// rh_p12_rn_base 아래 빈 자식 `tcp`(ROS 로컬 (0, 0, +0.236) m = Unity 로컬 (0, 0.236, 0)) 의 위치를
// 0.1초마다 LineRenderer 에 누적한다(최대 2000점). C 키로 지운다.
// 기본은 꺼짐, P 키로 켜고 끈다(10-21 결정: 단계 5 확인용 도구라 화면·영상에는 내지 않는다. 켜면 그때부터 새로 그린다).
// 꺼져 있어도 `tcp` 자식은 만든다 — 테스트·HUD 가 TCP 위치를 읽는다.
// `tcp` 자식은 SimSetup.BuildScene 이 씬에 만들어 두지만, 없으면 여기서 만든다.
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.UrdfImporter;

public class TcpTrail : MonoBehaviour
{
    public UrdfRobot robot;
    public string flangeLinkName = "rh_p12_rn_base";
    public string tcpChildName = "tcp";
    [Tooltip("ROS 로컬 (0,0,0.236) 을 임포터 변환 (x,y,z)→(-y,z,x) 으로 바꾼 Unity 로컬 오프셋")]
    public Vector3 tcpLocalOffset = new Vector3(0f, 0.236f, 0f);
    public float sampleInterval = 0.1f;
    public int maxPoints = 2000;
    public float lineWidth = 0.004f;
    public Color lineColor = new Color(0.1f, 0.9f, 1f, 0.9f);
    public KeyCode clearKey = KeyCode.C;
    [Tooltip("궤적 표시. 기본 꺼짐(P 로 켠다)")]
    public bool showTrail = false;
    public KeyCode toggleKey = KeyCode.P;

    public Transform Tcp { get; private set; }
    public int PointCount => m_Points.Count;
    public Vector3 TcpWorldPosition => Tcp != null ? Tcp.position : Vector3.zero;

    readonly List<Vector3> m_Points = new List<Vector3>();
    LineRenderer m_Line;
    float m_NextSample;

    void Start()
    {
        if (robot == null) robot = FindObjectOfType<UrdfRobot>();
        if (robot == null) { Debug.LogError("[TcpTrail] 씬에 UrdfRobot 이 없다"); enabled = false; return; }

        Transform flange = FindDeep(robot.transform, flangeLinkName);
        if (flange == null) { Debug.LogError($"[TcpTrail] 링크 '{flangeLinkName}' 없음"); enabled = false; return; }
        Tcp = flange.Find(tcpChildName);
        if (Tcp == null)
        {
            Tcp = new GameObject(tcpChildName).transform;
            Tcp.SetParent(flange, false);
            Tcp.localPosition = tcpLocalOffset;
            Tcp.localRotation = Quaternion.identity;
            Debug.Log($"[TcpTrail] '{tcpChildName}' 자식을 새로 만들었다 (BuildScene 이 안 만든 경우)");
        }

        // UnityEngine.Object 에 ?? 를 쓰면 안 된다: 에디터의 GetComponent 는 없는 컴포넌트에 '가짜 null'(접근 시 MissingComponentException)을 돌려준다
        m_Line = GetComponent<LineRenderer>();
        if (m_Line == null) m_Line = gameObject.AddComponent<LineRenderer>();
        m_Line.useWorldSpace = true;
        m_Line.widthMultiplier = lineWidth;
        m_Line.positionCount = 0;
        m_Line.numCapVertices = 2;
        m_Line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        m_Line.receiveShadows = false;
        var shader = Shader.Find("Sprites/Default");
        if (shader != null)
        {
            m_Line.material = new Material(shader);
            m_Line.startColor = m_Line.endColor = lineColor;
        }
        m_Line.enabled = showTrail;
        Debug.Log($"[TcpTrail] TCP 영점 world={Tcp.position:F4} (ROS (0,0,1.359) ↔ Unity y 1.359 기대, 시작 자세면 다름). 궤적 {(showTrail ? "켜짐" : "꺼짐")} (P)");
    }

    void Update()
    {
        if (Input.GetKeyDown(toggleKey)) { showTrail = !showTrail; Clear(); m_Line.enabled = showTrail; }
        if (Input.GetKeyDown(clearKey)) Clear();
        if (!showTrail) return;
        if (Time.timeSinceLevelLoad < m_NextSample) return;
        m_NextSample = Time.timeSinceLevelLoad + sampleInterval;

        m_Points.Add(Tcp.position);
        if (m_Points.Count > maxPoints) m_Points.RemoveAt(0);
        m_Line.positionCount = m_Points.Count;
        m_Line.SetPositions(m_Points.ToArray());
    }

    public void Clear()
    {
        m_Points.Clear();
        if (m_Line != null) m_Line.positionCount = 0;
    }

    static Transform FindDeep(Transform root, string name)
    {
        if (root.name == name) return root;
        foreach (Transform c in root) { var r = FindDeep(c, name); if (r != null) return r; }
        return null;
    }
}
