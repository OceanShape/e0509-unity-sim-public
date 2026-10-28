// 단계 8 화면 — Isaac HUD 를 같은 배치로 그린다 (딸기 리포 isaac_sim_viewport_display.py 의 패널과 그리퍼 카메라 창).
//
// 무엇을 그릴지는 scripts/hud_relay.py 가 딸기 리포 HUD 모듈(bus_merge·tree_model·result_bar·status_bus)로 계산해
// /tmp/e0509_hud_view.json (환경 변수 E0509_HUD_VIEW) 에 10 Hz 로 쓴다. 여기서는 읽어서 그리기만 한다 — 규칙은 딸기 리포 한 곳에 있다.
// 한글 글자는 Isaac 과 같은 라벨 PNG(assets_src/hud/labels → Resources/HudLabels, make_labels.py 가 2배 크기로 그린 것)이고,
// 숫자와 트리 노드 이름만 글꼴로 쓴다(Isaac 도 같다). 치수는 Isaac 1080p 화면의 논리 픽셀 × min(가로/1920, 세로/1080).
//   왼쪽 위   노드 램프 · 타겟 수 · 트리 · 단계 · 진행 막대 · 수확 결과. 뷰 파일이 없거나 데이터가 없으면 그리지 않는다(M1 bag 재생 등).
//   왼쪽 아래 그리퍼(D455) 카메라 창 + 보드 미니맵(팔이 작업 칸에 있을 때만, 0.3 s 페이드).
// 뷰 파일이 1.5 s 넘게 안 바뀌면(중계·노드 종료) 램프를 끈다. 나머지는 마지막 상태로 남는다(Isaac 도 마지막 스냅샷이 남는다).
// 이번 Play 를 시작하기 전에 쓰인 뷰(이전 런이 남긴 파일)는 없는 것으로 본다 — 10-21 발견: M1 bag 재생에서 이전 M2 런의
// '수확 완료' 패널이 램프 꺼진 채 떠 노드가 안 뜬 것처럼 보였다. Isaac 의 hud.install() 이 EPOCH 이전 파일을 버리는 것과 같은 규칙.
// 키: G — 패널·카메라 창 켜기/끄기.
// SIM_AUTOSHOT=<초> (에디터 그래픽 모드 Play): 그 간격마다, 그리고 ATTACH 1.5 s 뒤·RELEASE 1.0 s 뒤마다 Game 화면(HUD 포함)을
//   logs/render/autoshot/ 에 PNG 로 남긴다 — 사람 없이 화면을 확인하는 용도. 비우면 아무것도 안 한다.
using System;
using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;
using UnityEngine;

[Serializable] public class HudLamps { public bool vision, planner, controller, scan; }
[Serializable] public class HudCell { public string key; public int[] c; }
[Serializable] public class HudLegend { public string label; public int[] c; public int n; }
[Serializable] public class HudItem { public string k; public float x, y, w, h; public int[] fill, border, c; public float bw, r; public string t; public float s; public string a; public string img; public float alpha; }
[Serializable] public class HudMinimap { public bool show; public float w, h; public HudItem[] items; }
[Serializable]
public class HudView
{
    public double t; public bool have_data; public HudLamps lamps;
    public string state; public int[] state_color; public int bar_index, bar_len;
    public string harvest, final;
    public HudCell[] cells; public float cell_h, cell_gap, cell_r; public HudLegend[] legend; public float legend_size;
    public string area; public string[] lit; public string label_key;
    public float tree_w, tree_h; public HudItem[] tree; public HudMinimap minimap;
}

public class HarvestHud : MonoBehaviour
{
    public bool show = true;
    public KeyCode toggleKey = KeyCode.G;
    public GraspFollower follower;
    public WristCamera wristCamera;

    /// 마지막으로 읽은 뷰 모델 (BoardDisplay 도 읽는다). 파일이 없으면 null
    public static HudView View { get; private set; }
    /// 중계가 1.5 s 안에 썼는가
    public static bool ViewFresh => View != null && NowUnix() - View.t < 1.5;
    public static string ViewPath => Environment.GetEnvironmentVariable("E0509_HUD_VIEW") is string p && p.Length > 0 ? p : "/tmp/e0509_hud_view.json";
    public long ViewReads { get; private set; }

    // ── isaac_sim_viewport_display.py 의 치수·색 (딸기 HEAD 2797bb5) ──
    const float POS_X = 16, POS_Y = 32, PANEL_W = 440, PAD = 16, ROW_GAP = 12, HEAD_W = 60, LAMP_R = 8;
    const float CAM_POS_X = 16, CAM_MARGIN_BOTTOM = 16, CAM_W = 480, CAM_PAD = 10, CAM_IMG_H = 360;   // 480 × 3/4 (조리개 비)
    const float MM_PAD = 8, MM_RADIUS = 9, MM_MARGIN = 10, MM_FADE = 0.3f;
    static readonly Color C_BG = new Color(0.07f, 0.08f, 0.11f, 0.82f), C_LINE = new Color(1, 1, 1, 0.10f);
    static readonly Color C_TEXT = Hex(0xE8EDF5), C_DIM = Hex(0x7B8494), C_OK = Hex(0x5AD469), C_BAD = Hex(0xFF4D5E);
    static readonly Color C_SEG_OFF = new Color(1, 1, 1, 0.12f), C_SEG_DONE = new Color(1, 1, 1, 0.35f);
    static readonly Color C_MM_BADGE = new Color(0.063f, 0.071f, 0.102f, 0.80f);
    static readonly string[] k_Nodes = { "vision", "planner", "controller", "scan" };

    readonly Dictionary<string, Texture2D> m_Tex = new Dictionary<string, Texture2D>();
    readonly Dictionary<string, Vector2> m_Size = new Dictionary<string, Vector2>();
    float m_StateH = 58.5f, m_HarvestH = 49.5f;
    Font m_Font;
    readonly Dictionary<int, GUIStyle> m_Styles = new Dictionary<int, GUIStyle>();
    DateTime m_LastWrite;
    float m_NextPoll, m_MiniAlpha;
    float m_S = 1f;
    double m_SessionStart;
    bool m_StaleNoted;
    float m_ShotEvery, m_NextShot, m_EventShotAt = -1f;
    string m_ShotDir, m_EventShotName;
    int m_SeenAttach, m_SeenRelease, m_ShotCount;
    public int ShotCount => m_ShotCount;

    static Color Hex(int h, float a = 1f) => new Color(((h >> 16) & 255) / 255f, ((h >> 8) & 255) / 255f, (h & 255) / 255f, a);
    static Color C(int[] v) => v != null && v.Length >= 3 ? new Color(v[0] / 255f, v[1] / 255f, v[2] / 255f, v.Length > 3 ? v[3] / 255f : 1f) : Color.clear;
    static double NowUnix() => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;

    void Start()
    {
        if (follower == null) follower = FindObjectOfType<GraspFollower>();
        if (wristCamera == null) wristCamera = FindObjectOfType<WristCamera>();
        // 라벨 PNG 크기: manifest.json 의 w·h (논리 픽셀). 이름 → {"w": .., "h": ..}
        var man = Resources.Load<TextAsset>("HudLabels/manifest");
        if (man != null)
            foreach (Match m in Regex.Matches(man.text, "\"(\\w+)\":\\s*\\{\\s*\"w\":\\s*([0-9.]+),\\s*\"h\":\\s*([0-9.]+)"))
                m_Size[m.Groups[1].Value] = new Vector2(float.Parse(m.Groups[2].Value, System.Globalization.CultureInfo.InvariantCulture),
                                                        float.Parse(m.Groups[3].Value, System.Globalization.CultureInfo.InvariantCulture));
        foreach (var kv in m_Size)
        {
            if (kv.Key.StartsWith("state_")) m_StateH = Mathf.Max(m_StateH, kv.Value.y);
            if (kv.Key.StartsWith("harvest_")) m_HarvestH = Mathf.Max(m_HarvestH, kv.Value.y);
        }
        m_Font = Font.CreateDynamicFontFromOSFont(new[] { "Noto Sans CJK KR", "Noto Sans", "DejaVu Sans" }, 24);
        Debug.Log($"[HarvestHud] 라벨 {m_Size.Count}개 (Resources/HudLabels), 뷰 파일 {ViewPath}, 그리퍼 카메라 {(wristCamera != null ? "있음" : "없음")}. G 로 켜고 끈다");
        View = null;
        m_SessionStart = NowUnix();
        if (float.TryParse(Environment.GetEnvironmentVariable("SIM_AUTOSHOT"), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var every) && every > 0)
        {
            m_ShotEvery = every; m_NextShot = Time.unscaledTime + every;
            m_ShotDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "logs", "render", "autoshot"));
            Directory.CreateDirectory(m_ShotDir);
            Debug.Log($"[HarvestHud] SIM_AUTOSHOT {every:F0} s → {m_ShotDir}");
        }
    }

    void AutoShot()
    {
        if (m_ShotEvery <= 0) return;
        if (follower != null)
        {
            if (follower.AttachCount != m_SeenAttach) { m_SeenAttach = follower.AttachCount; m_EventShotAt = Time.unscaledTime + 1.5f; m_EventShotName = $"attach{m_SeenAttach}"; }
            if (follower.ReleaseCount != m_SeenRelease) { m_SeenRelease = follower.ReleaseCount; m_EventShotAt = Time.unscaledTime + 1.0f; m_EventShotName = $"release{m_SeenRelease}"; }
        }
        string name = null;
        if (m_EventShotAt > 0 && Time.unscaledTime >= m_EventShotAt) { name = m_EventShotName; m_EventShotAt = -1f; }
        else if (Time.unscaledTime >= m_NextShot) { name = $"t{Time.unscaledTime:000}"; m_NextShot += m_ShotEvery; }
        if (name == null) return;
        string file = Path.Combine(m_ShotDir, $"{DateTime.Now:HHmmss}_{name}_{(View != null ? View.state : "noview")}.png");
        ScreenCapture.CaptureScreenshot(file);
        m_ShotCount++;
        Debug.Log($"[HarvestHud] 화면 저장 → {file} ({Screen.width}×{Screen.height})");
    }

    void Update()
    {
        if (Input.GetKeyDown(toggleKey)) show = !show;
        AutoShot();
        float target = View != null && View.have_data && View.minimap != null && View.minimap.show ? 1f : 0f;
        m_MiniAlpha = Mathf.MoveTowards(m_MiniAlpha, target, Time.unscaledDeltaTime / MM_FADE);
        if (Time.unscaledTime < m_NextPoll) return;
        m_NextPoll = Time.unscaledTime + 0.1f;
        try
        {
            var fi = new FileInfo(ViewPath);
            if (!fi.Exists) { View = null; return; }
            if (fi.LastWriteTimeUtc == m_LastWrite) return;
            m_LastWrite = fi.LastWriteTimeUtc;
            var v = JsonUtility.FromJson<HudView>(File.ReadAllText(ViewPath));
            if (v != null && v.t < m_SessionStart - 2.0)
            {
                if (!m_StaleNoted) Debug.Log($"[HarvestHud] 뷰 파일이 이번 Play 전에 쓰였다({m_SessionStart - v.t:F0} s 전) — 이전 런의 것이라 그리지 않는다");
                m_StaleNoted = true;
                View = null;
                return;
            }
            View = v;
            ViewReads++;
        }
        catch (Exception) { /* 중계가 쓰는 중이면 다음 주기에 다시 (os.replace 라 드물다) */ }
    }

    // ── 그리기 도우미 (논리 픽셀 → 화면 픽셀 m_S 배) ──
    // 가장자리를 화면 정수 픽셀에 맞춘다. Isaac 은 1080p 정수 좌표라 선이 또렷한데, 배율(m_S)을 곱한 소수 좌표를 그대로 쓰면
    // 가장자리가 반투명하게 번져 미니맵 테두리가 두 줄처럼 보였다(10-21 발견). 양 끝을 따로 반올림해 이웃 도형과 틈·겹침이 없다.
    Rect R(float x, float y, float w, float h)
    {
        float x0 = Mathf.Round(x * m_S), y0 = Mathf.Round(y * m_S);
        float x1 = Mathf.Round((x + w) * m_S), y1 = Mathf.Round((y + h) * m_S);
        if (w > 0 && x1 - x0 < 1) x1 = x0 + 1;
        if (h > 0 && y1 - y0 < 1) y1 = y0 + 1;
        return new Rect(x0, y0, x1 - x0, y1 - y0);
    }
    void Box(float x, float y, float w, float h, Color c, float radius = 0, float border = 0)
    {
        if (c.a <= 0f || w <= 0 || h <= 0) return;
        var r = R(x, y, w, h);
        if (radius <= 0 && border <= 0)
        {
            // 모서리 없는 단색 사각형(선·칸)은 둥근 사각형 셰이더(가장자리 부드럽게)를 거치지 않는다
            var prev = GUI.color; GUI.color = c;
            GUI.DrawTexture(r, Texture2D.whiteTexture);
            GUI.color = prev;
            return;
        }
        GUI.DrawTexture(r, Texture2D.whiteTexture, ScaleMode.StretchToFill, true, 0, c, border * m_S, radius * m_S);
    }
    Vector2 LabelSize(string name) => m_Size.TryGetValue(name, out var v) ? v : Vector2.zero;
    Texture2D Tex(string name)
    {
        if (!m_Tex.TryGetValue(name, out var t)) { t = Resources.Load<Texture2D>("HudLabels/" + name); m_Tex[name] = t; }
        return t;
    }
    // 라벨 PNG 를 (x, 줄 위 y, 줄 높이 rowH) 에 세로 가운데로. 그린 폭을 돌려준다
    float Img(string name, float x, float y, float rowH, float alpha = 1f)
    {
        var sz = LabelSize(name); var t = Tex(name);
        if (t == null || sz.x <= 0) return 0;
        GUI.DrawTexture(R(x, y + (rowH - sz.y) / 2f, sz.x, sz.y), t, ScaleMode.ScaleToFit, true, 0, new Color(1, 1, 1, alpha), 0, 0);
        return sz.x;
    }
    GUIStyle Style(float size, TextAnchor anchor)
    {
        int px = Mathf.Max(6, Mathf.RoundToInt(size * m_S));
        int key = px * 16 + (int)anchor;
        if (!m_Styles.TryGetValue(key, out var st))
        {
            st = new GUIStyle(GUI.skin.label) { font = m_Font, fontSize = px, alignment = anchor, wordWrap = false, clipping = TextClipping.Overflow, padding = new RectOffset(0, 0, 0, 0), margin = new RectOffset(0, 0, 0, 0) };
            m_Styles[key] = st;
        }
        return st;
    }
    float TextW(string s, float size) => Style(size, TextAnchor.MiddleLeft).CalcSize(new GUIContent(s)).x / m_S;
    void Text(string s, float x, float y, float w, float h, float size, Color c, TextAnchor anchor = TextAnchor.MiddleLeft)
    {
        var st = Style(size, anchor); st.normal.textColor = c;
        GUI.Label(R(x, y, w, h), s, st);
    }

    void OnGUI()
    {
        if (!show || Event.current.type != EventType.Repaint) return;
        m_S = Mathf.Min(Screen.width / 1920f, Screen.height / 1080f);
        if (View != null && View.have_data) DrawPanel(View);
        if (wristCamera != null && wristCamera.Texture != null) DrawWristInset(View);
    }

    // ── 왼쪽 위 패널 (_build 의 줄 순서: 노드 · 타겟 수 · 구분선 · 트리 · 구분선 · 단계 · 막대 · 결과) ──
    void DrawPanel(HudView v)
    {
        float inner = PANEL_W - 2 * PAD;
        float hNodes = Mathf.Max(LabelSize("head_nodes").y, LabelSize("node_vision").y, LAMP_R * 2 + 2);
        float hCounts = Mathf.Max(LabelSize("count_targets").y, 30f);
        float hTree = Mathf.Max(LabelSize("head_tree").y, v.tree_h);
        float hPhase = Mathf.Max(LabelSize("head_phase").y, m_StateH);
        float hBar = 10;
        float hLegend = Mathf.Max(LabelSize("final_placed").y, v.legend_size * 1.4f);
        float hResult = 1 + ROW_GAP + m_HarvestH + ROW_GAP + v.cell_h + ROW_GAP + hLegend;
        float total = PAD + hNodes + hCounts + 1 + hTree + 1 + hPhase + hBar + hResult + 7 * ROW_GAP + PAD;
        Box(POS_X, POS_Y, PANEL_W, total, C_BG, 10);
        float x0 = POS_X + PAD, y = POS_Y + PAD;
        bool fresh = ViewFresh;

        // 노드: 머리말 + 램프 4개(초록 살아 있음, 빨강 없음)
        Img("head_nodes", x0, y, hNodes);
        float x = x0 + HEAD_W;
        foreach (var n in k_Nodes)
        {
            bool alive = fresh && v.lamps != null && (n == "vision" ? v.lamps.vision : n == "planner" ? v.lamps.planner : n == "controller" ? v.lamps.controller : v.lamps.scan);
            float d = LAMP_R * 2, cell = LAMP_R * 2 + 2;
            Box(x + (cell - d) / 2, y + (hNodes - d) / 2, d, d, alive ? C_OK : C_BAD, LAMP_R);
            x += cell + 5;
            x += Img("node_" + n, x, y, hNodes) + 10;
        }
        y += hNodes + ROW_GAP;

        // 타겟 N / 비타겟 M (씬의 과실 수 — Isaac 도 씬 프림을 센다)
        string nT = follower != null ? follower.Fruits.Count.ToString() : "-", nN = follower != null ? follower.UnripeCount.ToString() : "-";
        x = x0;
        x += Img("count_targets", x, y, hCounts) + 6;
        Text(nT, x, y, 60, hCounts, 24, C_TEXT); x += TextW(nT, 24) + 6 + 4 + 6;
        Text("/", x, y, 20, hCounts, 24, C_DIM); x += TextW("/", 24) + 6 + 4 + 6;
        x += Img("count_non_targets", x, y, hCounts) + 6;
        Text(nN, x, y, 60, hCounts, 24, C_DIM);
        y += hCounts + ROW_GAP;

        Box(x0, y, inner, 1, C_LINE); y += 1 + ROW_GAP;

        // 트리 (tree_model.view + node_paint 가 정한 도형 목록을 그대로)
        Img("head_tree", x0, y, LabelSize("head_tree").y);
        if (v.tree != null) DrawItems(v.tree, x0 + HEAD_W, y, 1f);
        y += hTree + ROW_GAP;

        Box(x0, y, inner, 1, C_LINE); y += 1 + ROW_GAP;

        // 단계: 머리말 오른쪽 폭의 가운데에 단계 라벨(색은 PNG 에 들어 있다)
        Img("head_phase", x0, y, hPhase);
        string sn = "state_" + (v.state ?? "IDLE");
        float sw = LabelSize(sn).x;
        Img(sn, x0 + HEAD_W + (inner - HEAD_W - sw) / 2, y, hPhase);
        y += hPhase + ROW_GAP;

        // 진행 막대 11칸: 지금 칸 = 단계 색, 지난 칸 = 흐린 흰색
        int nb = Mathf.Max(1, v.bar_len);
        float segW = (inner - HEAD_W - 3 * (nb - 1)) / nb;
        Color sc = C(v.state_color);
        for (int i = 0; i < nb; i++)
            Box(x0 + HEAD_W + i * (segW + 3), y, segW, hBar, i == v.bar_index ? sc : i < v.bar_index ? C_SEG_DONE : C_SEG_OFF, 2);
        y += hBar + ROW_GAP;

        // 결과: 구분선 · 수확 상태(+완료 수치) · 결과 막대(타겟마다 한 칸) · 범례
        Box(x0, y, inner, 1, C_LINE); y += 1 + ROW_GAP;
        string hn = "harvest_" + (v.harvest ?? "idle");
        bool fin = !string.IsNullOrEmpty(v.final);
        float hw = LabelSize(hn).x + (fin ? 12 + TextW(v.final, 34) : 0);
        x = x0 + (inner - hw) / 2;
        x += Img(hn, x, y, m_HarvestH);
        if (fin) Text(v.final, x + 12, y, 300, m_HarvestH, 34, C_TEXT);
        y += m_HarvestH + ROW_GAP;
        int nc = v.cells != null ? v.cells.Length : 0;
        if (nc > 0)
        {
            float cw = (inner - v.cell_gap * (nc - 1)) / nc;
            for (int i = 0; i < nc; i++)
                Box(x0 + i * (cw + v.cell_gap), y, cw, v.cell_h, string.IsNullOrEmpty(v.cells[i].key) ? C_SEG_OFF : C(v.cells[i].c), v.cell_r);
        }
        y += v.cell_h + ROW_GAP;
        if (v.legend != null)
        {
            float lw = 0;
            for (int i = 0; i < v.legend.Length; i++)
                lw += (i > 0 ? 4 + 10 + 4 : 0) + LabelSize(v.legend[i].label).x + 4 + TextW(v.legend[i].n.ToString(), v.legend_size);
            x = x0 + (inner - lw) / 2;
            for (int i = 0; i < v.legend.Length; i++)
            {
                if (i > 0) x += 4 + 10 + 4;
                x += Img(v.legend[i].label, x, y, hLegend) + 4;
                string num = v.legend[i].n.ToString();
                Text(num, x, y, 60, hLegend, v.legend_size, C(v.legend[i].c));
                x += TextW(num, v.legend_size);
            }
        }
    }

    // 트리·미니맵 도형 목록: rect(채움+테두리, 모서리 r) · fill(사각) · text(글꼴) · img(라벨 PNG, 투명도)
    void DrawItems(HudItem[] items, float ox, float oy, float alpha)
    {
        foreach (var it in items)
        {
            switch (it.k)
            {
                case "rect":
                    Box(ox + it.x, oy + it.y, it.w, it.h, Fade(C(it.fill), alpha), it.r);
                    if (it.bw > 0) Box(ox + it.x, oy + it.y, it.w, it.h, Fade(C(it.border), alpha), it.r, it.bw);
                    break;
                case "fill":
                    Box(ox + it.x, oy + it.y, it.w, it.h, Fade(C(it.c), alpha));
                    break;
                case "text":
                    Text(it.t, ox + it.x, oy + it.y, it.w, it.h, it.s, Fade(C(it.c), alpha),
                         it.a == "c" ? TextAnchor.MiddleCenter : it.a == "r" ? TextAnchor.MiddleRight : TextAnchor.MiddleLeft);
                    break;
                case "img":
                {
                    var sz = LabelSize(it.img); var t = Tex(it.img);
                    if (t != null) GUI.DrawTexture(R(ox + it.x, oy + it.y, sz.x, sz.y), t, ScaleMode.ScaleToFit, true, 0, new Color(1, 1, 1, it.alpha * alpha), 0, 0);
                    break;
                }
            }
        }
    }
    static Color Fade(Color c, float a) { c.a *= a; return c; }

    // ── 왼쪽 아래 그리퍼 카메라 창 (_WristCamera.build): 제목 라벨 · 640×480 렌더를 480×360 으로 · 오른쪽 위 보드 미니맵 ──
    void DrawWristInset(HudView v)
    {
        float titleH = LabelSize("cam_title").y;
        float h = CAM_PAD + titleH + 6 + CAM_IMG_H + CAM_PAD, w = CAM_W + 2 * CAM_PAD;
        float x0 = CAM_POS_X, y0 = Screen.height / m_S - CAM_MARGIN_BOTTOM - h;
        Box(x0, y0, w, h, C_BG, 10);
        Img("cam_title", x0 + CAM_PAD, y0 + CAM_PAD, titleH);
        float ix = x0 + CAM_PAD, iy = y0 + CAM_PAD + titleH + 6;
        GUI.DrawTexture(R(ix, iy, CAM_W, CAM_IMG_H), wristCamera.Texture, ScaleMode.StretchToFill, false);
        if (m_MiniAlpha <= 0.001f || v == null || v.minimap == null || v.minimap.items == null) return;
        float mw = v.minimap.w, mh = v.minimap.h;
        float bx = ix + CAM_W - MM_MARGIN - (mw + 2 * MM_PAD), by = iy + MM_MARGIN;
        Box(bx, by, mw + 2 * MM_PAD, mh + 2 * MM_PAD, Fade(C_MM_BADGE, m_MiniAlpha), MM_RADIUS);
        DrawItems(v.minimap.items, bx + MM_PAD, by + MM_PAD, m_MiniAlpha);
    }
}
