// 단계 8 화면 — 그리퍼 손목 D455 컬러 카메라 (Isaac rsd455/RSD455/Camera_OmniVision_OV9782_Color 와 같은 자리·화각).
// SimEnvironment 가 scene.json 의 wrist_camera 값으로 rh_p12_rn_base 아래에 놓고 수직 화각 63.83°(640×480 → 가로 79.41°)를 넣는다.
// 화면에 직접 그리지 않고 640×480 RenderTexture 에 그린다 → HarvestHud 가 왼쪽 아래 창에 480×360 으로 보인다(Isaac 과 같은 크기).
// D455 본체 메쉬(bodyLayer)는 렌즈 앞(유리·마스크)을 가리므로 이 카메라에서만 뺀다. 인식(YOLO)은 없다 — 렌더만이다(Isaac 창 제목과 같다).
// 그래픽 장치가 없으면(batchmode -nographics: 헤드리스 시험·M2 플랜트) 끈다 — 화면 밖 카메라 렌더가 Null 장치에서
// 그림자 맵을 그리다 죽었다(10-21 M1 bag 시험, RenderManager::RenderOffscreenCameras 에서 SIGSEGV). 보는 화면이 없으니 잃는 것도 없다.
using UnityEngine;

[RequireComponent(typeof(Camera))]
public class WristCamera : MonoBehaviour
{
    public int width = 640, height = 480;
    public int bodyLayer = 30;
    public RenderTexture Texture { get; private set; }
    Camera m_Cam;

    void Awake()
    {
        m_Cam = GetComponent<Camera>();
        if (SystemInfo.graphicsDeviceType == UnityEngine.Rendering.GraphicsDeviceType.Null)
        {
            m_Cam.enabled = false;
            Debug.Log("[WristCamera] 그래픽 장치 없음(-nographics) — 손목 카메라 렌더 끔");
            return;
        }
        Texture = new RenderTexture(width, height, 24) { antiAliasing = 4, name = "WristCamera" };
        m_Cam.targetTexture = Texture;
        m_Cam.cullingMask &= ~(1 << bodyLayer);
        Debug.Log($"[WristCamera] {width}×{height}, 수직 화각 {m_Cam.fieldOfView:F2}° (가로 {2f * Mathf.Atan(Mathf.Tan(m_Cam.fieldOfView * 0.5f * Mathf.Deg2Rad) * width / height) * Mathf.Rad2Deg:F2}°), 부모 {(transform.parent != null ? transform.parent.name : "-")}");
    }

    void OnDestroy()
    {
        if (m_Cam != null) m_Cam.targetTexture = null;
        if (Texture != null) { Texture.Release(); Destroy(Texture); }
    }
}
