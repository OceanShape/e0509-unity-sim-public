// 단계 5 — 진단 패널(오른쪽 위). OnGUI. I 키로 켜고 끈다.
// 단계 8 부터 화면의 주 HUD 는 Isaac 과 같은 HarvestHud(왼쪽 위)이고, 이 패널은 Unity 쪽 사실을 보여 준다:
//   ROS 연결·모드 → 루프 속도(명령 수신·상태 발행 Hz, 추종 오차) → 한계 위반·근처·계란판·바닥 → 잡는 중·발행 과실 수 → 관절각(측정).
// 10-25 재구성: 한 줄짜리 모노스페이스 7줄이 1080p 에서도 읽기 어려워(글자 크기 하나, 강조 없음, 왼쪽 패널과 다른 모양)
//   왼쪽 HarvestHud 와 같은 카드(같은 배경·여백·글꼴·흐린 라벨 + 큰 값)로 다시 그린다. 보는 사람이 판단할 수치는 큰 타일,
//   관절각은 작고 흐리게. 라벨은 한글(Noto Sans CJK KR 이 없으면 영문), 토픽 이름·단위는 영문.
// 10-25 고정 칸: 값이 바뀔 때 옆 항목이 좌우로 밀리지 않게, 모든 항목을 고정 열(3열 타일, 4열 격자)에 둔다. 값 뒤에 붙는 단위는
//   값 자리 폭(예: "00.0")을 미리 잡아 그 뒤 고정 위치에 쓴다. 패널 폭·높이는 상태와 무관하게 같다 — 맨 아래 경고 줄도 늘 한 줄을
//   잡아 두고(없으면 흐린 '경고 없음'), 경고가 여럿이면 한 줄에 이어 쓰며 넘치면 개수만 쓴다.
//   세로 위치는 글자 기준선으로 맞춘다 — IMGUI 의 Middle 정렬은 Noto Sans CJK 의 큰 줄 높이 때문에 글자가 상자 가운데보다
//   0.25 em 아래에 놓였다(10-25 캡처 측정: 16 px 알약에서 4 px 아래).
// 기본: Mirror(M1, Isaac HUD 데이터 없음) 켜짐, Command(M2) 꺼짐. 커넥터 자체 HUD 는 끈다(왼쪽 위가 Isaac 패널 자리) — 연결 상태는 여기 머리줄.
// 치수는 1080p 논리 픽셀 × min(가로/1920, 세로/1080). 작은 Game 뷰에서는 minScale 아래로 줄이지 않는다(10-22: 고정 13 px 이 너무 작았다).
using System.Collections.Generic;
using UnityEngine;

public class SimHud : MonoBehaviour
{
    public JointStateMirror mirror;
    public JointLimitMonitor limitMonitor;
    public TcpTrail tcpTrail;
    public JointStatePublisher statePub;
    public StrawberryPublisher berryPub;
    public GraspFollower follower;
    [Tooltip("화면이 작아도 이 배율(1080p 기준) 아래로는 줄이지 않는다")]
    public float minScale = 0.65f;
    public KeyCode toggleKey = KeyCode.I;
    [Tooltip("-1 = 모드로 정함(Mirror 켜짐, Command 꺼짐), 0 꺼짐, 1 켜짐")]
    public int showMode = -1;
    bool m_Show = true;

    // ── HarvestHud 와 같은 치수·색 (논리 픽셀) ──
    const float MARGIN_RIGHT = 16, POS_Y = 32, PANEL_W = 440, PAD = 16, ROW_GAP = 12, COL_GAP = 10;
    const float INNER = PANEL_W - 2 * PAD;
    const float TILE_W = (INNER - 2 * COL_GAP) / 3f;          // 3열 타일
    const float CELL_PITCH = (INNER + COL_GAP) / 4f, CELL_W = CELL_PITCH - COL_GAP;   // 4열 격자(작은 타일·잡기 줄·관절각이 같은 열)
    const float H_HEAD = 34, H_SUB = 18, H_TILE = 68, H_MINI = 50, H_LINE = 26, H_JOINT = 24, H_WARN = 22, CHIP_H = 26, DOT = 14;
    const float F_HEAD = 22, F_CHIP = 16, F_LABEL = 15, F_VALUE = 30, F_UNIT = 16, F_SUB = 14, F_MINI = 26, F_LINE = 16, F_JOINT = 16;
    // Noto Sans CJK, Middle 정렬에서 기준선 = 상자 가운데 + K_BASE·em, 글자(대문자·한글) 가운데 = 기준선 − K_INK·em (10-25 캡처 측정)
    const float K_BASE = 0.625f, K_INK = 0.40f;
    static readonly Color C_BG = new Color(0.07f, 0.08f, 0.11f, 0.82f), C_LINE = new Color(1, 1, 1, 0.10f), C_CHIP = new Color(1, 1, 1, 0.10f);
    static readonly Color C_TEXT = Hex(0xE8EDF5), C_DIM = Hex(0x7B8494), C_OK = Hex(0x5AD469), C_BAD = Hex(0xFF4D5E), C_WARN = Hex(0xFFB547);
    static readonly string[] k_JointNames = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "rh_p12_rn" };

    Font m_Font;
    bool m_Korean;
    readonly Dictionary<int, GUIStyle> m_Styles = new Dictionary<int, GUIStyle>();
    readonly List<string> m_Warns = new List<string>();
    float m_S = 1f;

    static Color Hex(int h, float a = 1f) => new Color(((h >> 16) & 255) / 255f, ((h >> 8) & 255) / 255f, (h & 255) / 255f, a);

    void Start()
    {
        // UnityEngine.Object 비교는 == null 로 (?? 는 가짜 null 을 통과시킨다)
        if (mirror == null) mirror = GetComponent<JointStateMirror>();
        if (mirror == null) mirror = FindObjectOfType<JointStateMirror>();
        if (limitMonitor == null) limitMonitor = GetComponent<JointLimitMonitor>();
        if (limitMonitor == null) limitMonitor = FindObjectOfType<JointLimitMonitor>();
        if (tcpTrail == null) tcpTrail = GetComponent<TcpTrail>();
        if (tcpTrail == null) tcpTrail = FindObjectOfType<TcpTrail>();
        if (statePub == null) statePub = FindObjectOfType<JointStatePublisher>();
        if (berryPub == null) berryPub = FindObjectOfType<StrawberryPublisher>();
        if (follower == null) follower = FindObjectOfType<GraspFollower>();
        m_Show = showMode >= 0 ? showMode == 1 : !(mirror != null && mirror.mode == JointStateMirror.ApplyMode.Command);
        // 한글 가능 여부는 OS 글꼴 목록으로 본다 — Font.HasCharacter 는 동적 글꼴에서 글리프를 요청해 Start 가 멈췄다(10-25 복사본 시험, 메인 스레드 100%).
        bool cjk = System.Array.IndexOf(Font.GetOSInstalledFontNames(), "Noto Sans CJK KR") >= 0;
        m_Font = Font.CreateDynamicFontFromOSFont(cjk ? new[] { "Noto Sans CJK KR", "Noto Sans", "DejaVu Sans" } : new[] { "Noto Sans", "DejaVu Sans" }, 24);
        m_Korean = cjk && m_Font != null;
        Debug.Log($"[SimHud] 진단 패널 글꼴 {(m_Font != null ? m_Font.name : "기본")}, 한글 {(m_Korean ? "가능" : "없음 → 영문 라벨")}. {toggleKey} 로 켜고 끈다");
    }

    void Update()
    {
        if (Input.GetKeyDown(toggleKey)) m_Show = !m_Show;
    }

    // ── 그리기 도우미 (HarvestHud 와 같은 규칙: 논리 픽셀 → 화면 픽셀 m_S 배, 가장자리는 정수 픽셀) ──
    Rect R(float x, float y, float w, float h)
    {
        float x0 = Mathf.Round(x * m_S), y0 = Mathf.Round(y * m_S);
        float x1 = Mathf.Round((x + w) * m_S), y1 = Mathf.Round((y + h) * m_S);
        if (w > 0 && x1 - x0 < 1) x1 = x0 + 1;
        if (h > 0 && y1 - y0 < 1) y1 = y0 + 1;
        return new Rect(x0, y0, x1 - x0, y1 - y0);
    }
    void Box(float x, float y, float w, float h, Color c, float radius = 0)
    {
        if (c.a <= 0f || w <= 0 || h <= 0) return;
        var r = R(x, y, w, h);
        if (radius <= 0) { var prev = GUI.color; GUI.color = c; GUI.DrawTexture(r, Texture2D.whiteTexture); GUI.color = prev; return; }
        GUI.DrawTexture(r, Texture2D.whiteTexture, ScaleMode.StretchToFill, true, 0, c, 0, radius * m_S);
    }
    GUIStyle Style(float size, TextAnchor anchor)
    {
        int px = Mathf.Max(6, Mathf.RoundToInt(size * m_S));
        int key = px * 16 + (int)anchor;
        if (!m_Styles.TryGetValue(key, out var st))
        {
            st = new GUIStyle(GUI.skin.label) { fontSize = px, alignment = anchor, wordWrap = false, clipping = TextClipping.Overflow, padding = new RectOffset(0, 0, 0, 0), margin = new RectOffset(0, 0, 0, 0) };
            if (m_Font != null) st.font = m_Font;
            m_Styles[key] = st;
        }
        return st;
    }
    float TextW(string s, float size) => Style(size, TextAnchor.MiddleLeft).CalcSize(new GUIContent(s)).x / m_S;
    static TextAnchor Middle(TextAnchor a) => a == TextAnchor.MiddleCenter || a == TextAnchor.MiddleRight ? a : TextAnchor.MiddleLeft;
    // 기준선을 baseY 에 맞춰 쓴다(크기가 다른 값·단위를 한 줄에 맞출 때)
    void TextBase(string s, float x, float baseY, float w, float size, Color c, TextAnchor anchor = TextAnchor.MiddleLeft)
    {
        var st = Style(size, Middle(anchor)); st.normal.textColor = c;
        float center = baseY - K_BASE * size;
        GUI.Label(R(x, center - size, w, 2 * size), s, st);
    }
    // 글자를 (y, h) 띠의 세로 가운데에 쓴다
    void Text(string s, float x, float y, float w, float h, float size, Color c, TextAnchor anchor = TextAnchor.MiddleLeft)
        => TextBase(s, x, y + h / 2f + K_INK * size, w, size, c, anchor);
    string L(string ko, string en) => m_Korean ? ko : en;

    // 타일: 흐린 라벨(위) · 큰 값 + 작은 단위 · 흐린 보조 글(아래).
    // slot 이 있으면 값은 그 폭(예: "00.0") 안에 오른쪽 정렬하고 단위는 그 뒤 고정 위치 — 자릿수가 바뀌어도 단위가 안 움직이고 값에 붙어 있다.
    void Tile(float x, float y, string label, string value, string slot, string unit, Color valueColor, string sub)
    {
        Text(label, x, y, TILE_W, 18, F_LABEL, C_DIM);
        float baseY = y + 47;
        if (string.IsNullOrEmpty(slot)) TextBase(value, x, baseY, TILE_W, F_VALUE, valueColor);
        else
        {
            float sw = TextW(slot, F_VALUE);
            TextBase(value, x, baseY, sw, F_VALUE, valueColor, TextAnchor.MiddleRight);
            if (!string.IsNullOrEmpty(unit)) TextBase(unit, x + sw + 4, baseY, TILE_W - sw - 4, F_UNIT, C_DIM);
        }
        if (!string.IsNullOrEmpty(sub)) Text(sub, x, y + 52, TILE_W, 16, F_SUB, C_DIM);
    }
    // 작은 타일(4열 격자 한 칸): 흐린 라벨 · 값 · 값 자리("00") 뒤 고정 위치의 덧붙임(빨강 등)
    void Mini(int col, float x0, float y, string label, string value, Color valueColor, string extra = null, Color extraColor = default)
    {
        float x = x0 + col * CELL_PITCH;
        Text(label, x, y, CELL_W, 18, F_LABEL, C_DIM);
        float baseY = y + 44;
        TextBase(value, x, baseY, CELL_W, F_MINI, valueColor);
        if (!string.IsNullOrEmpty(extra))
        {
            float ex = x + Mathf.Max(TextW("00", F_MINI), TextW(value, F_MINI)) + 8;
            TextBase(extra, ex, baseY, CELL_W - (ex - x), F_SUB, extraColor);
        }
    }

    void OnGUI()
    {
        if (!m_Show || Event.current.type != EventType.Repaint) return;
        m_S = Mathf.Max(minScale, Mathf.Min(Screen.width / 1920f, Screen.height / 1080f));
        bool cmd = mirror != null && mirror.mode == JointStateMirror.ApplyMode.Command;

        // ── 머리줄 상태 ──
        string state, stateEn; Color stateColor; string ep = "—";
        if (mirror == null) { state = "JointStateMirror 없음"; stateEn = "JointStateMirror missing"; stateColor = C_BAD; }
        else
        {
            var ros = mirror.Ros;
            if (ros == null)                  { state = "ROS 미시작";      stateEn = "ROS NOT STARTED"; stateColor = C_BAD; }
            else if (ros.HasConnectionError)  { state = "ROS 2 연결 오류"; stateEn = "ROS 2 ERROR";     stateColor = C_BAD; }
            else if (ros.HasConnectionThread) { state = "ROS 2 연결됨";    stateEn = "ROS 2 CONNECTED"; stateColor = C_OK; }
            else                              { state = "ROS 2 연결 안 됨"; stateEn = "ROS 2 DISCONNECTED"; stateColor = C_WARN; }
            if (ros != null) ep = $"{ros.RosIPAddress}:{ros.RosPort}";
        }
        string chipM1 = L("M1 · 미러", "M1 · MIRROR"), chipM2 = L("M2 · 명령 루프", "M2 · COMMAND");

        // 경고 줄(늘 한 줄 — 경고가 생겨도 패널 높이가 안 바뀐다). 여럿이면 ' · ' 로 잇고, 넘치면 관절 이름 대신 개수
        m_Warns.Clear();
        int nUnknown = mirror != null ? mirror.UnknownNames.Count : 0;
        if (nUnknown > 0) m_Warns.Add(L("알 수 없는 관절: ", "Unknown joints: ") + string.Join(", ", mirror.UnknownNames));
        if (follower != null && follower.IgnoredCount > 0) m_Warns.Add(L($"무시한 잡기 이벤트 {follower.IgnoredCount}", $"Ignored grasp events {follower.IgnoredCount}"));
        string warn = string.Join(" · ", m_Warns);
        if (nUnknown > 0 && TextW(warn, F_SUB) > INNER)
        {
            m_Warns[0] = L($"알 수 없는 관절 {nUnknown}개", $"Unknown joints ×{nUnknown}");
            warn = string.Join(" · ", m_Warns);
        }

        float total = PAD + H_HEAD + H_SUB + ROW_GAP + 1 + ROW_GAP + H_TILE + ROW_GAP + H_MINI + 4 + H_LINE
                    + ROW_GAP + 1 + ROW_GAP + 2 * H_JOINT + 4 + H_WARN + PAD;
        float panelX = Screen.width / m_S - MARGIN_RIGHT - PANEL_W;
        Box(panelX, POS_Y, PANEL_W, total, C_BG, 10);
        float x0 = panelX + PAD, y = POS_Y + PAD;

        // ── 머리줄: 색 점 · 연결 상태 · 모드 알약(두 모드 중 긴 글자에 맞춘 고정 폭) / 엔드포인트 주소 ──
        Box(x0, y + (H_HEAD - DOT) / 2, DOT, DOT, stateColor, DOT / 2);
        Text(L(state, stateEn), x0 + DOT + 10, y, INNER - DOT - 10, H_HEAD, F_HEAD, stateColor);
        float cw = Mathf.Max(TextW(chipM1, F_CHIP), TextW(chipM2, F_CHIP)) + 24;
        float cy = y + (H_HEAD - CHIP_H) / 2;
        Box(x0 + INNER - cw, cy, cw, CHIP_H, C_CHIP, CHIP_H / 2);
        Text(cmd ? chipM2 : chipM1, x0 + INNER - cw, cy, cw, CHIP_H, F_CHIP, C_TEXT, TextAnchor.MiddleCenter);
        y += H_HEAD;
        Text(L("엔드포인트 ", "endpoint ") + ep, x0 + DOT + 10, y, INNER, H_SUB, F_SUB, C_DIM);
        y += H_SUB + ROW_GAP; Box(x0, y, INNER, 1, C_LINE); y += 1 + ROW_GAP;

        // ── 3열 타일: 루프 속도 ──
        float rx = mirror != null ? mirror.ReceiveHz : 0f;
        Color rxColor = cmd ? (rx > 0f ? C_OK : C_DIM) : (rx >= 50f ? C_OK : rx > 0f ? C_WARN : C_BAD);
        string rxTopic = mirror != null ? mirror.topic : "-";
        float t1 = x0, t2 = x0 + TILE_W + COL_GAP, t3 = x0 + 2 * (TILE_W + COL_GAP);
        bool haveJoints = mirror != null && mirror.Joints.Count > 0;
        if (cmd)
        {
            Tile(t1, y, L("명령 수신", "Command in"), $"{rx:F1}", "00.0", "Hz", rxColor, rxTopic);
            bool pubOn = statePub != null && statePub.Active;
            float tx = pubOn ? statePub.PublishHz : 0f;
            Tile(t2, y, L("상태 발행", "State out"), pubOn ? $"{tx:F1}" : "—", "00.0", "Hz", pubOn ? (tx >= 55f ? C_OK : C_WARN) : C_DIM,
                 statePub != null ? statePub.topic : "-");
            float err = 0f;
            if (haveJoints)
                foreach (var kv in mirror.Joints)
                    err = Mathf.Max(err, Mathf.Abs(Mathf.DeltaAngle(kv.Value.jointPosition[0] * Mathf.Rad2Deg, kv.Value.xDrive.target)));
            Tile(t3, y, L("추종 오차", "Track error"), haveJoints ? $"{err:F2}°" : "—", null, null, C_TEXT, L("명령−측정, 최대", "cmd−measured, max"));
        }
        else
        {
            Tile(t1, y, L("수신", "Receive"), $"{rx:F1}", "00.0", "Hz", rxColor, rxTopic);
            double since = mirror != null ? Time.realtimeSinceStartupAsDouble - mirror.LastReceiveRealtime : double.PositiveInfinity;
            bool none = double.IsInfinity(since) || since > 1e6;
            // 1 s 넘게 끊기면 초로(값 자리 "000" 을 넘지 않게)
            string sv = none ? "—" : since < 1.0 ? $"{since * 1000.0:F0}" : since < 100.0 ? $"{since:F1}" : "99+";
            Tile(t2, y, L("마지막 수신", "Last message"), sv, "000", none ? "" : since < 1.0 ? "ms" : "s",
                 none ? C_DIM : since > 0.5 ? C_WARN : C_TEXT, none ? L("아직 없음", "none yet") : null);
            Tile(t3, y, L("메시지", "Messages"), mirror != null ? mirror.MessageCount.ToString("N0") : "—", null, null, C_TEXT, L("받은 수", "received"));
        }
        y += H_TILE + ROW_GAP;

        // ── 4열 작은 타일: 한계 위반(+ 지금 위반 중 관절) · 근처 · 계란판 · 바닥 ──
        if (limitMonitor != null)
        {
            int v = limitMonitor.ViolationCount;
            var now = limitMonitor.ActiveViolationJoints;
            string nowStr = now.Length == 0 ? null : L("지금 ", "now ") + now[0].Replace("joint_", "J") + (now.Length > 1 ? $" +{now.Length - 1}" : "");
            Mini(0, x0, y, L("한계 위반", "Limit viol."), v.ToString(), v > 0 ? C_BAD : C_OK, nowStr, C_BAD);
            Mini(1, x0, y, L("한계 근처", "Near limit"), limitMonitor.NearCount.ToString(), C_TEXT);
        }
        else { Mini(0, x0, y, L("한계 위반", "Limit viol."), "—", C_DIM); Mini(1, x0, y, L("한계 근처", "Near limit"), "—", C_DIM); }
        if (follower != null)
        {
            Mini(2, x0, y, L("계란판", "In tray"), follower.PlacedCount.ToString(), C_TEXT);
            Mini(3, x0, y, L("바닥", "On floor"), follower.DroppedCount.ToString(), follower.DroppedCount > 0 ? C_WARN : C_TEXT);
        }
        else { Mini(2, x0, y, L("계란판", "In tray"), "—", C_DIM); Mini(3, x0, y, L("바닥", "On floor"), "—", C_DIM); }
        y += H_MINI + 4;

        // ── 잡기 줄: 잡는 중(1–2열) · 발행 과실(3–4열, Command 만). 라벨 폭이 고정이라 값 위치도 고정 ──
        string heldLabel = L("잡는 중", "Holding");
        string held = follower != null && follower.Attached != null ? follower.Attached.name.Replace("strawberry_", "") : "—";
        Text(heldLabel, x0, y, CELL_PITCH + CELL_W, H_LINE, F_LINE, C_DIM);
        Text(held, x0 + TextW(heldLabel, F_LINE) + 10, y, CELL_PITCH + CELL_W, H_LINE, F_LINE, held == "—" ? C_DIM : C_TEXT);
        if (cmd)
        {
            string bl = L("발행 과실", "Berries out"), bv = berryPub != null && berryPub.Active ? $"{Mathf.Max(0, berryPub.LastPublishedCount)}/{berryPub.ripeRos.Length}" : "—";
            float bx = x0 + 2 * CELL_PITCH;
            Text(bl, bx, y, CELL_PITCH + CELL_W, H_LINE, F_LINE, C_DIM);
            Text(bv, bx + TextW(bl, F_LINE) + 10, y, CELL_W, H_LINE, F_LINE, bv == "—" ? C_DIM : C_TEXT);
        }
        y += H_LINE;

        // ── 관절각(측정): 4열 × 2행, 작은 타일과 같은 열, 값은 칸 오른쪽 끝 정렬 ──
        y += ROW_GAP; Box(x0, y, INNER, 1, C_LINE); y += 1 + ROW_GAP;
        string[] labels = { "J1", "J2", "J3", "J4", "J5", "J6", L("그리퍼", "Grip") };
        for (int i = 0; i < k_JointNames.Length; i++)
        {
            float cx = x0 + (i % 4) * CELL_PITCH, cyy = y + (i / 4) * H_JOINT;
            string val = haveJoints && mirror.Joints.TryGetValue(k_JointNames[i], out var ab) ? $"{ab.jointPosition[0] * Mathf.Rad2Deg:F1}°" : "—";
            Text(labels[i], cx, cyy, CELL_W, H_JOINT, F_JOINT, C_DIM);
            // 그리퍼 칸(둘째 줄 셋째)은 라벨이 길어 넷째 칸까지 쓴다 — 값이 J4 열과 같은 오른쪽 끝에 맞는다
            float vw = i == 6 ? CELL_PITCH + CELL_W : CELL_W;
            Text(val, cx, cyy, vw, H_JOINT, F_JOINT, C_TEXT, TextAnchor.MiddleRight);
        }
        y += 2 * H_JOINT;

        y += 4;
        if (m_Warns.Count == 0) Text(L("경고 없음", "No warnings"), x0, y, INNER, H_WARN, F_SUB, C_DIM);
        else Text(warn, x0, y, INNER, H_WARN, F_SUB, C_WARN);
    }
}
