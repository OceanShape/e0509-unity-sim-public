// 화면용 카메라 조작 (단계 8). Game 뷰에서 마우스로 로봇을 가까이 본다. Isaac 뷰포트처럼:
//   오른쪽 드래그  — 회전(중심점 둘레)          가운데 드래그(또는 Shift+오른쪽) — 이동
//   휠             — 확대·축소(중심점 쪽으로)    F — 중심점을 TCP 로   T — TCP 따라가기 켜기/끄기
//   1–9            — 구도 프리셋(1 Isaac 롱샷 = 시작 구도, 2 녹화 근접, 3 측면, 4·5 짝 구도, 6 로봇 근접)
//   R              — 시작 구도로                 H — 도움말 표시 켜기/끄기
// 프리셋은 scripts/export_isaac_scene.py 가 Isaac Kit 확장 설정(extension.toml, record_camera.py)에서 계산한 값이다.
// 화각은 Isaac 처럼 **가로**를 고정한다: 물리 카메라(센서 가로 20.955 mm = Isaac horizontalAperture, 초점거리 프리셋별),
// Gate Fit = Horizontal. 그래서 Game 창 비율이 16:9 가 아니어도(4:3, 세로로 긴 창) 좌우로 보이는 범위가 Isaac 과 같다.
// (10-21 전까지는 세로 화각을 고정해, 창이 16:9 보다 좁으면 롱샷에서 책상 양끝이 잘렸다.)
using System;
using System.Collections.Generic;
using UnityEngine;

public class SimCameraController : MonoBehaviour
{
    [Serializable]
    public class Preset
    {
        public string name;
        public Vector3 position;
        public Vector3 forward;
        public Vector3 up = Vector3.up;
        [Tooltip("16:9 일 때의 세로 화각(참고값). 실제 화각은 focalLength + 센서 가로(가로 맞춤)로 정한다")]
        public float verticalFov = 40f;
        [Tooltip("초점거리 mm (Isaac focalLength). 0 이면 verticalFov 를 쓴다(세로 맞춤)")]
        public float focalLength = 0f;
    }

    /// Isaac 카메라 horizontalAperture (mm) — 모든 Isaac 구도가 같은 값
    public float sensorWidthMm = 20.955f;

    public List<Preset> presets = new List<Preset>();
    public int startPreset = 0;
    [Tooltip("회전 중심을 정할 때 쓰는 작업 영역 중심(Unity 좌표). 로봇과 보드 사이")]
    public Vector3 workspaceCenter = new Vector3(-0.45f, 0.75f, 0f);
    public float orbitSpeed = 0.25f;     // deg / pixel
    public float panSpeed = 0.0015f;     // 거리 비례
    public float zoomStep = 0.12f;
    public bool showHelp = true;

    Camera m_Cam;
    Vector3 m_Pivot;
    bool m_FollowTcp;
    Transform m_Tcp;
    int m_Current;
    Vector3 m_LastMouse;
    GUIStyle m_Style;

    void Start()
    {
        m_Cam = GetComponent<Camera>();
        var tcpGo = GameObject.Find("tcp");
        if (tcpGo != null) m_Tcp = tcpGo.transform;
        if (presets.Count > 0) ApplyPreset(Mathf.Clamp(startPreset, 0, presets.Count - 1));
        else m_Pivot = workspaceCenter;
    }

    public void ApplyPreset(int i)
    {
        if (i < 0 || i >= presets.Count) return;
        var p = presets[i];
        transform.position = p.position;
        transform.rotation = Quaternion.LookRotation(p.forward, p.up);
        if (m_Cam != null) ApplyLens(m_Cam, p, sensorWidthMm);
        // 회전 중심: 시선 위에서 작업 영역 중심에 가장 가까운 점
        Vector3 f = p.forward.normalized;
        float t = Mathf.Max(0.3f, Vector3.Dot(workspaceCenter - p.position, f));
        m_Pivot = p.position + f * t;
        m_Current = i;
        m_FollowTcp = false;
    }

    /// Isaac 과 같은 렌즈: 가로 센서 폭 고정 + 초점거리 → 가로 화각 고정(Gate Fit Horizontal). RenderPreview 도 쓴다.
    public static void ApplyLens(Camera cam, Preset p, float sensorWidthMm)
    {
        if (p.focalLength > 0f)
        {
            cam.usePhysicalProperties = true;
            cam.sensorSize = new Vector2(sensorWidthMm, sensorWidthMm * 9f / 16f);
            cam.gateFit = Camera.GateFitMode.Horizontal;
            cam.lensShift = Vector2.zero;
            cam.focalLength = p.focalLength;
        }
        else
        {
            cam.usePhysicalProperties = false;
            cam.fieldOfView = p.verticalFov;
        }
    }

    void LateUpdate()
    {
        // 숫자 키 프리셋
        for (int i = 0; i < Mathf.Min(9, presets.Count); i++)
            if (Input.GetKeyDown(KeyCode.Alpha1 + i) || Input.GetKeyDown(KeyCode.Keypad1 + i)) ApplyPreset(i);
        if (Input.GetKeyDown(KeyCode.R)) ApplyPreset(Mathf.Clamp(startPreset, 0, Mathf.Max(0, presets.Count - 1)));
        if (Input.GetKeyDown(KeyCode.H)) showHelp = !showHelp;
        if (Input.GetKeyDown(KeyCode.F) && m_Tcp != null) FocusOn(m_Tcp.position);
        if (Input.GetKeyDown(KeyCode.T) && m_Tcp != null) { m_FollowTcp = !m_FollowTcp; if (m_FollowTcp) FocusOn(m_Tcp.position); }

        if (m_FollowTcp && m_Tcp != null)
        {
            Vector3 d = m_Tcp.position - m_Pivot;
            transform.position += d; m_Pivot += d;
        }

        Vector3 mouse = Input.mousePosition;
        Vector3 delta = mouse - m_LastMouse;
        m_LastMouse = mouse;
        bool pan = Input.GetMouseButton(2) || (Input.GetMouseButton(1) && (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift)));
        bool orbit = Input.GetMouseButton(1) && !pan;
        if (Input.GetMouseButtonDown(1) || Input.GetMouseButtonDown(2)) delta = Vector3.zero;

        if (orbit && delta.sqrMagnitude > 0)
        {
            transform.RotateAround(m_Pivot, Vector3.up, delta.x * orbitSpeed);
            // 위아래: 뒤집히지 않게 제한
            float pitch = -delta.y * orbitSpeed;
            float angle = Vector3.Angle(transform.forward, Vector3.up);
            if (angle - pitch < 5f) pitch = angle - 5f;
            if (angle - pitch > 175f) pitch = angle - 175f;
            transform.RotateAround(m_Pivot, transform.right, pitch);
        }
        if (pan && delta.sqrMagnitude > 0)
        {
            float dist = Vector3.Distance(transform.position, m_Pivot);
            Vector3 move = (-transform.right * delta.x - transform.up * delta.y) * panSpeed * dist;
            transform.position += move; m_Pivot += move;
            m_FollowTcp = false;
        }
        float wheel = Input.mouseScrollDelta.y;
        if (Mathf.Abs(wheel) > 0.01f)
        {
            float dist = Vector3.Distance(transform.position, m_Pivot);
            float nd = Mathf.Clamp(dist * (1f - zoomStep * Mathf.Sign(wheel)), 0.08f, 20f);
            transform.position = m_Pivot - transform.forward * nd;
        }
    }

    void FocusOn(Vector3 p)
    {
        float dist = Mathf.Clamp(Vector3.Distance(transform.position, m_Pivot), 0.25f, 0.8f);
        m_Pivot = p;
        transform.position = p - transform.forward * dist;
    }

    void OnGUI()
    {
        if (!showHelp) return;
        if (m_Style == null)
        {
            m_Style = new GUIStyle(GUI.skin.label) { fontSize = 12, alignment = TextAnchor.LowerRight };   // 왼쪽 아래는 그리퍼 카메라 창
            m_Style.normal.textColor = new Color(1f, 1f, 1f, 0.85f);
        }
        string names = "";
        for (int i = 0; i < Mathf.Min(9, presets.Count); i++) names += $"{i + 1}:{presets[i].name}{(i == m_Current ? "*" : "")}  ";
        string text = "Camera  RMB orbit · MMB/Shift+RMB pan · wheel zoom · F focus TCP · T follow TCP" + (m_FollowTcp ? " [ON]" : "") +
                      " · R reset · H hide · G HUD · I diag · P trail\n" + names;
        GUI.Label(new Rect(10, Screen.height - 44, Screen.width - 20, 40), text, m_Style);   // 오른쪽 아래 정렬
    }
}
