// 단계 8 화면 — 보드 하이라이트(청록 테두리 20장)와 보드 방위 글자 텍스처(9장)를 HUD 뷰 모델대로 바꾼다.
// Isaac: _BoardHighlight(테두리 프림 visibility 켜고 끄기) · _BoardTexture(보드 재질 diffuse_texture 를 board_<key>.png 로 바꿈).
// 무엇을 켤지는 hud_relay.py 가 딸기 HUD 모듈(highlight_key · HIGHLIGHT_ON · tree_model.board_label_key)로 정한 lit · label_key 다.
// 뷰 파일이 없으면(M1 bag 재생 등) 테두리를 다 끄고 보드는 원래 격자 텍스처 — Isaac 에서 노드 없이 씬만 열었을 때와 같다.
// 보드 텍스처는 MaterialPropertyBlock 으로만 바꾼다(재질 에셋을 고치지 않는다 — Isaac 이 세션 레이어에만 쓰는 것과 같은 이유).
using System;
using UnityEngine;

public class BoardDisplay : MonoBehaviour
{
    public Renderer board;
    public string[] highlightNames = new string[0];
    public GameObject[] highlights = new GameObject[0];
    public string[] labelKeys = new string[0];
    public Texture2D[] labelTextures = new Texture2D[0];

    public string LitNow { get; private set; } = "";
    public string LabelNow { get; private set; } = "";
    string m_Lit = null, m_Label = null;
    MaterialPropertyBlock m_Mpb;

    void Start() => Apply(new string[0], "");

    void Update()
    {
        var v = HarvestHud.View;
        bool have = v != null && v.have_data;
        Apply(have && v.lit != null ? v.lit : new string[0], have ? (v.label_key ?? "") : "");
    }

    public void Apply(string[] lit, string labelKey)
    {
        string sig = string.Join(",", lit);
        if (sig != m_Lit)
        {
            for (int i = 0; i < highlights.Length && i < highlightNames.Length; i++)
                if (highlights[i] != null) highlights[i].SetActive(Array.IndexOf(lit, highlightNames[i]) >= 0);
            m_Lit = sig; LitNow = sig;
        }
        if (labelKey != m_Label && board != null)
        {
            int k = Array.IndexOf(labelKeys, labelKey);
            if (k >= 0 && k < labelTextures.Length && labelTextures[k] != null)
            {
                if (m_Mpb == null) m_Mpb = new MaterialPropertyBlock();
                board.GetPropertyBlock(m_Mpb);
                m_Mpb.SetTexture("_MainTex", labelTextures[k]);
                board.SetPropertyBlock(m_Mpb);
            }
            else board.SetPropertyBlock(null);     // 키 없음 = 씬의 원래 격자
            m_Label = labelKey; LabelNow = k >= 0 ? labelKey : "";
        }
    }
}
