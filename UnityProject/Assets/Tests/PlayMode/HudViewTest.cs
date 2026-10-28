// HUD 뷰 모델 시험 (단계 8 화면). scripts/hud_relay.py 가 쓴 JSON 을 HarvestHud 가 읽고(JsonUtility), BoardDisplay 가 보드 테두리·방위 글자를 바꾸는지.
// 입력: HUD_VIEW_SAMPLE (hud_relay.py --once 결과 파일). 없으면 Ignore.
//   E0509_HUD_EPOCH=0 python3 -B scripts/hud_relay.py --once --out /tmp/x.json && HUD_VIEW_SAMPLE=/tmp/x.json …
// 확인: Play 전에 쓰인 뷰 무시, 파싱(트리 도형·결과 칸·범례·미니맵), 램프 신선도(파일 시각), 테두리 켜기(분면·세부 칸), 방위 글자 교체, 키 없음 = 원래 격자.
using System;
using System.Collections;
using System.IO;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

public class HudViewTest
{
    string m_Path;

    IEnumerator Write(HudView v)
    {
        File.WriteAllText(m_Path, JsonUtility.ToJson(v));
        File.SetLastWriteTimeUtc(m_Path, DateTime.UtcNow.AddSeconds(UnityEngine.Random.Range(1, 1000)));   // 같은 초 안에 다시 써도 바뀐 것으로 보이게
        yield return new WaitForSecondsRealtime(0.35f);
    }

    [UnityTest]
    public IEnumerator RelayView_Parses_And_DrivesBoard()
    {
        string sample = Environment.GetEnvironmentVariable("HUD_VIEW_SAMPLE");
        if (string.IsNullOrEmpty(sample) || !File.Exists(sample)) Assert.Ignore("HUD_VIEW_SAMPLE 없음");
        m_Path = Path.Combine(Application.temporaryCachePath, "hud_view_test.json");
        // 표본 그대로(옛 시각) → HarvestHud 가 '이번 Play 전 파일' 로 버려야 한다
        File.Copy(sample, m_Path, true);
        Environment.SetEnvironmentVariable("E0509_HUD_VIEW", m_Path);
        JointStateMirror.ForcedMode = JointStateMirror.ApplyMode.Command;
        JointStateMirror.SkipRos = true;
        yield return SceneManager.LoadSceneAsync("Assets/Scenes/E0509Sim.unity", LoadSceneMode.Single);
        yield return new WaitForSecondsRealtime(0.4f);
        Assert.IsNull(HarvestHud.View, "이전 런이 남긴(Play 전에 쓰인) 뷰는 그리지 않아야 한다");

        // 같은 내용을 지금 시각으로 다시 쓴다 = 이번 세션의 중계 출력
        var fresh = JsonUtility.FromJson<HudView>(File.ReadAllText(sample));
        fresh.t = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;
        yield return Write(fresh);

        var v = HarvestHud.View;
        Assert.IsNotNull(v, "HarvestHud 가 뷰 파일을 못 읽었다");
        Assert.IsTrue(v.have_data);
        Assert.Greater(v.tree.Length, 5, "트리 도형 목록");
        Assert.IsTrue(v.tree.Any(i => i.k == "rect") && v.tree.Any(i => i.k == "text"), "트리에 사각형·글자가 있어야 한다");
        Assert.AreEqual(3, v.legend.Length, "범례 3개(배치·배치 실패·분리 실패)");
        Assert.AreEqual(11, v.bar_len, "진행 막대 11칸");
        Assert.IsNotNull(v.minimap); Assert.Greater(v.minimap.items.Length, 0);
        Debug.Log($"[HudViewTest] 상태 {v.state}, 수확 {v.harvest} '{v.final}', 칸 {v.cells.Length}, 트리 도형 {v.tree.Length}, 켜짐 [{string.Join(",", v.lit)}], 글자 '{v.label_key}'");

        var bd = UnityEngine.Object.FindObjectOfType<BoardDisplay>();
        Assert.IsNotNull(bd, "BoardDisplay 가 없다");
        Assert.AreEqual(20, bd.highlights.Length, "하이라이트 20장");
        Assert.AreEqual(string.Join(",", v.lit), bd.LitNow, "보드 테두리가 뷰의 lit 과 다르다");
        Assert.AreEqual(v.label_key, bd.LabelNow, "보드 방위 글자가 뷰의 label_key 와 다르다");

        // 세부 칸 하나 + 2단계 글자
        v.lit = new[] { "ne_sw" }; v.label_key = "q2_ne"; v.t = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;
        yield return Write(v);
        Assert.AreEqual("ne_sw", bd.LitNow);
        Assert.AreEqual(1, bd.highlights.Count(h => h.activeSelf), "켜진 테두리는 하나");
        Assert.IsTrue(bd.highlights[Array.IndexOf(bd.highlightNames, "ne_sw")].activeSelf);
        Assert.AreEqual("q2_ne", bd.LabelNow);
        var mpb = new MaterialPropertyBlock(); bd.board.GetPropertyBlock(mpb);
        var tex = mpb.GetTexture("_MainTex");
        Assert.IsNotNull(tex, "보드 텍스처 덮어쓰기가 없다");
        Assert.AreEqual("board_q2_ne", tex.name, "보드 텍스처가 q2_ne 로 바뀌지 않았다");
        Assert.IsTrue(HarvestHud.ViewFresh, "방금 쓴 뷰는 신선해야 한다(램프 켜짐)");

        // 키 없음 → 원래 격자, 테두리 없음. 오래된 시각 → 램프 끔
        v.lit = new string[0]; v.label_key = ""; v.t -= 10;
        yield return Write(v);
        Assert.AreEqual("", bd.LitNow);
        Assert.AreEqual(0, bd.highlights.Count(h => h.activeSelf));
        Assert.AreEqual("", bd.LabelNow);
        Assert.IsFalse(bd.board.HasPropertyBlock(), "키가 없으면 보드 원래 텍스처(속성 블록 없음)");
        Assert.IsFalse(HarvestHud.ViewFresh, "10 s 전 뷰는 신선하지 않다(램프 끔)");

        // 파일이 없으면 View = null → 테두리 끔
        File.Delete(m_Path);
        yield return new WaitForSecondsRealtime(0.35f);
        Assert.IsNull(HarvestHud.View);
    }

    [TearDown]
    public void TearDown()
    {
        JointStateMirror.ForcedMode = null; JointStateMirror.SkipRos = false;
        Environment.SetEnvironmentVariable("E0509_HUD_VIEW", null);
        if (m_Path != null && File.Exists(m_Path)) File.Delete(m_Path);
    }
}
