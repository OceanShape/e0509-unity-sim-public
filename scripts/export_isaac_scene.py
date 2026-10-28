#!/usr/bin/env python3
"""Isaac 씬의 정적 형상·재질·조명·카메라 구도를 Unity 용으로 내보낸다 (단계 8 화면, 딸기 리포 읽기 전용).

원천: ~/strawberry_grasp_environment/strawberry_harvest/scenes/main_scene.usd (pxr 로 읽기만 한다)
결과: assets_src/scene/
  scene.json      메쉬(Unity 좌표, 삼각형, 법선·UV), 인스턴스 배치(TRS), 상자, 재질, 조명, 카메라 프리셋, 출처
  textures/*.png  딸기·보드 텍스처 (딸기 원본 20–25 MB 를 1024 px 로 줄임)
SimSetup.BuildScene 이 이 파일로 Environment 오브젝트를 만든다(콜라이더 없음).

좌표: Isaac/ROS 는 Z-up 오른손 m. Unity 는 URDF-Importer 와 같은 변환 C: (x, y, z) → (−y, z, x) (행렬식 −1).
  메쉬 정점·법선은 C 를 곱하고 삼각형 감김을 뒤집는다. 배치 행렬 M(로컬→월드, ROS)은 C·M·C⁻¹ 로 바꿔 TRS 로 나눈다.
빼는 것: 로봇(URDF 로 따로 들어옴), 보드 하이라이트(Isaac HUD 가 켜고 끄는 발광 사각형), 그리퍼 카메라,
  Isaac 에서 보이지 않는 형상(visibility invisible, 예: board_guard 충돌 상자).
  그리퍼의 D455 본체는 NVIDIA 자산이라 형상 대신 부품별 경계 상자만 낸다(THIRD_PARTY.md §2-1).
검사: 딸기 메쉬 중심이 CONTRACT §3-1 좌표(StrawberryPublisher 가 내는 값)와 몇 mm 안인지, 보드면 y = 0.810 인지 출력한다.
  python3 -B scripts/export_isaac_scene.py [--src ~/strawberry_grasp_environment] [--tex 1024]
"""
import argparse
import datetime
import hashlib
import json
import math
import os
import re
import subprocess
import sys

import numpy as np
from PIL import Image
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
C = np.array([[0, -1, 0], [0, 0, 1], [1, 0, 0]], dtype=np.float64)       # ROS → Unity
CONTRACT_RIPE = {f"strawberry_ripe_{i:02d}": p for i, p in enumerate(
    [(-0.25, 0.7828, 0.74), (-0.12, 0.7828, 0.76), (0.20, 0.7828, 0.74), (-0.22, 0.7828, 0.56),
     (-0.10, 0.7828, 0.44), (-0.25, 0.7828, 0.50), (-0.38, 0.7828, 0.72), (0.28, 0.7828, 0.70)], start=1)}


def sha256(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def r5(a):
    return [round(float(v), 5) for v in np.asarray(a).reshape(-1)]


def mat4(prim):
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    # Gf 는 행 벡터 규약(p' = p·M). 열 벡터 규약으로 전치
    return np.array([[m[i][j] for j in range(4)] for i in range(4)], dtype=np.float64).T


def to_unity_trs(M):
    """ROS 로컬→월드 4×4 (열 벡터) → Unity 위치·회전(쿼터니언 x,y,z,w)·스케일."""
    R = M[:3, :3]; t = M[:3, 3]
    U = C @ R @ C.T                       # C 는 직교라 C⁻¹ = Cᵀ
    pos = C @ t
    sx, sy, sz = (np.linalg.norm(U[:, i]) for i in range(3))
    Rn = U / np.array([sx, sy, sz])
    if np.linalg.det(Rn) < 0:             # 반사가 남으면 x 스케일에 넘긴다
        sx = -sx; Rn[:, 0] = -Rn[:, 0]
    return {"pos": r5(pos), "rot": r5(quat_from_matrix(Rn)), "scale": r5([sx, sy, sz])}


def quat_from_matrix(R):
    """열 벡터 회전행렬 → 쿼터니언 (x, y, z, w). Unity 의 Quaternion * v 와 같은 규약."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s; x = (R[2, 1] - R[1, 2]) / s; y = (R[0, 2] - R[2, 0]) / s; z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s; y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s; y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s; y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    return [x, y, z, w]


def mesh_unity(prim, world=False, ref=None):
    """UsdGeom.Mesh → 삼각형 펼친 배열(Unity 좌표). world=True 면 월드로, ref(4×4, ROS 월드) 를 주면 그 프레임 기준,
    아니면 프림 로컬 좌표."""
    m = UsdGeom.Mesh(prim)
    P = np.array(m.GetPointsAttr().Get(), dtype=np.float64)
    fvc = list(m.GetFaceVertexCountsAttr().Get())
    fvi = list(m.GetFaceVertexIndicesAttr().Get())
    if ref is not None:
        M = np.linalg.inv(ref) @ mat4(prim)
        world = True
    elif world:
        M = mat4(prim)
    if world:
        P = (M[:3, :3] @ P.T).T + M[:3, 3]
    pv = UsdGeom.PrimvarsAPI(prim)
    st = pv.GetPrimvar("st")
    uv, uv_interp, uv_idx = None, None, None
    if st and st.HasValue():
        uv = np.array(st.Get(), dtype=np.float64)
        uv_interp = st.GetInterpolation()
        uv_idx = st.GetIndices() if st.IsIndexed() else None
    nattr = m.GetNormalsAttr()
    N, n_interp = None, None
    if nattr and nattr.HasValue():
        N = np.array(nattr.Get(), dtype=np.float64); n_interp = m.GetNormalsInterpolation()
        if world:
            N = (np.linalg.inv(M[:3, :3]).T @ N.T).T
    pos, nor, uvs = [], [], []
    corner = 0
    for face, n in enumerate(fvc):
        idx = fvi[corner:corner + n]
        cs = list(range(corner, corner + n))
        for k in range(1, n - 1):                      # 부채꼴 삼각분할
            tri = [0, k, k + 1]
            for j in (0, 2, 1):                        # 감김 반전(반사 변환)
                c = cs[tri[j]]; v = idx[tri[j]]
                pos.append(C @ P[v])
                if N is not None:
                    nv = N[c] if n_interp == "faceVarying" else (N[face] if n_interp == "uniform" else N[v])
                    nor.append(C @ (nv / (np.linalg.norm(nv) or 1)))
                if uv is not None:
                    if uv_interp == "faceVarying":
                        u = uv[uv_idx[c]] if uv_idx is not None else uv[c]
                    else:
                        u = uv[uv_idx[v]] if uv_idx is not None else uv[v]
                    uvs.append(u)
        corner += n
    out = {"positions": r5(pos), "triangles": len(pos) // 3}
    if nor:
        out["normals"] = r5(nor)
    if uvs:
        out["uvs"] = r5(uvs)
    return out, P


def bbox_box(geo):
    """부품을 그 축정렬 경계 상자(같은 좌표계, 삼각형 12개)로 바꾼다. 앞면은 바깥쪽 — Unity 는 cross(b−a, c−a) 쪽이 앞면이다.
    D455 본체는 NVIDIA Isaac Sim 자산이라 형상을 옮기지 않고 부품마다 크기·자리만 남긴다(THIRD_PARTY.md §2-1).
    법선·UV 는 내지 않는다(Unity 가 면마다 다시 계산)."""
    P = np.array(geo["positions"]).reshape(-1, 3)
    lo, hi = P.min(0), P.max(0)
    ctr = (lo + hi) / 2
    c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    tris = []
    for q in ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)):   # 면마다 꼭짓점 4개를 둘레 순서로
        for t in ((q[0], q[1], q[2]), (q[0], q[2], q[3])):
            a, b, d = c[list(t)]
            if np.dot(np.cross(b - a, d - a), (a + b + d) / 3 - ctr) < 0:
                b, d = d, b
            tris += [a, b, d]
    return {"positions": r5(tris), "triangles": 12}


def double_sided(geo):
    """삼각형마다 감김을 뒤집은 뒷면을 더한다(법선 반전, UV 그대로)."""
    P = np.array(geo["positions"]).reshape(-1, 3, 3)
    back = P[:, ::-1, :]
    g = {"positions": r5(np.concatenate([P, back]).reshape(-1)), "triangles": geo["triangles"] * 2}
    # 법선은 넣지 않는다 — 원본 법선이 감김과 어긋나면 한쪽 면이 뒤에서 비친다. 정점이 모서리마다 따로라
    # Unity RecalculateNormals 가 삼각형마다(앞면·뒷면 각자 자기 감김으로) 평평한 법선을 만든다.
    if "uvs" in geo:
        U = np.array(geo["uvs"]).reshape(-1, 3, 2)
        g["uvs"] = r5(np.concatenate([U, U[:, ::-1, :]]).reshape(-1))
    return g


def shader_inputs(mat):
    res = {}
    if not mat:
        return res
    for prim in Usd.PrimRange(mat.GetPrim()):
        if not prim.IsA(UsdShade.Shader):
            continue
        sh = UsdShade.Shader(prim)
        for i in sh.GetInputs():
            v = i.Get()
            if i.HasConnectedSource():
                src = i.GetConnectedSource()[0]
                for j in UsdShade.Shader(src.GetPrim()).GetInputs():
                    if j.GetBaseName() == "file" and isinstance(j.Get(), Sdf.AssetPath):
                        res[i.GetBaseName() + "_texture"] = j.Get().resolvedPath or j.Get().path
            elif isinstance(v, Sdf.AssetPath):
                res[i.GetBaseName()] = v.resolvedPath or v.path
            elif v is not None:
                res[i.GetBaseName()] = v
    return res


def color3(v, default=(0.6, 0.6, 0.6)):
    try:
        return r5([v[0], v[1], v[2]])
    except Exception:
        return list(default)


def copy_texture(src, out_dir, name, size, normal=False):
    os.makedirs(out_dir, exist_ok=True)
    dst = os.path.join(out_dir, name)
    im = Image.open(src)
    im = im.convert("RGB")
    if max(im.size) > size:
        im.thumbnail((size, size), Image.LANCZOS)
    im.save(dst, optimize=True)
    return {"file": os.path.relpath(dst, ROOT), "source": src, "source_sha256": sha256(src), "size": list(im.size), "normal": normal}


def camera_preset(name, translate, rotate_xyz, focal, h_ap, aspect=16 / 9):
    """Kit 확장의 persp 카메라 설정(translate + rotateXYZ, 초점·조리개 mm) → Unity 위치·전방·위·세로 FOV."""
    stage = Usd.Stage.CreateInMemory()
    cam = UsdGeom.Camera.Define(stage, "/cam")
    x = UsdGeom.Xformable(cam)
    x.AddTranslateOp().Set(Gf.Vec3d(*translate))
    x.AddRotateXYZOp().Set(Gf.Vec3f(*rotate_xyz))
    M = mat4(cam.GetPrim())
    fwd = -M[:3, 2]; up = M[:3, 1]                    # USD 카메라는 −Z 를 본다
    v_ap = h_ap / aspect
    return {"name": name, "pos": r5(C @ M[:3, 3]), "forward": r5(C @ fwd), "up": r5(C @ up),
            "vfov_deg": round(math.degrees(2 * math.atan(v_ap / 2 / focal)), 3),
            "focal_mm": float(focal), "h_aperture_mm": float(h_ap),
            "isaac": {"translate": translate, "rotate_xyz": rotate_xyz, "focal_mm": focal, "h_aperture_mm": h_ap}}


def toml_presets(path):
    txt = open(path, encoding="utf-8").read()
    def arr(key):
        m = re.search(r'^\s*exts\."strawberry\.sim\.setup"\.' + re.escape(key) + r'\s*=\s*\[([^\]]*)\]', txt, re.M)
        return [float(v) for v in m.group(1).split(",")] if m else None
    def num(key):
        m = re.search(r'^\s*exts\."strawberry\.sim\.setup"\.' + re.escape(key) + r'\s*=\s*([-0-9.]+)', txt, re.M)
        return float(m.group(1)) if m else None
    out = {"long": (arr("persp_translate"), arr("persp_rotate_xyz"), num("persp_focal_length"), num("persp_horizontal_aperture"))}
    for p in ("side_s", "pair_a2", "pair_a3"):
        out[p] = (arr(f"persp_presets.{p}.translate"), arr(f"persp_presets.{p}.rotate_xyz"),
                  num(f"persp_presets.{p}.focal_length"), num(f"persp_presets.{p}.horizontal_aperture"))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=os.path.expanduser("~/strawberry_grasp_environment"))
    ap.add_argument("--tex", type=int, default=1024)
    a = ap.parse_args()
    src = os.path.realpath(a.src)
    scene = os.path.join(src, "strawberry_harvest/scenes/main_scene.usd")
    out_dir = os.path.join(ROOT, "assets_src", "scene")
    tex_dir = os.path.join(out_dir, "textures")
    if out_dir.startswith(src):
        sys.exit("!! 결과 폴더가 딸기 리포 안이다")
    st = Usd.Stage.Open(scene)
    head = subprocess.run(["git", "--no-optional-locks", "-C", src, "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()

    out = {"source": {"scene": scene, "strawberry_repo_head": head, "scene_sha256": sha256(scene),
                      "exported": datetime.datetime.now().isoformat(timespec="seconds"),
                      "conversion": "Unity = (-y, z, x) of ROS/Isaac; triangle winding reversed"},
           "meshes": {}, "materials": {}, "objects": [], "boxes": [], "lights": [], "cameras": [], "checks": {}}

    # ── 재질 ──
    def add_material(key, mat, fallback_color=None):
        if key in out["materials"]:
            return key
        ins = shader_inputs(mat)
        d = {"color": color3(ins.get("diffuse_color_constant", ins.get("diffuseColor", fallback_color or (0.6, 0.6, 0.6)))),
             "smoothness": round(1.0 - float(ins.get("reflection_roughness_constant", ins.get("roughness", 0.6)) if not isinstance(ins.get("roughness"), str) else 0.6), 3),
             "metallic": 0.0}
        for k_src, k_dst, normal in (("diffuse_texture", "albedo", False), ("diffuseColor_texture", "albedo", False),
                                     ("normal_texture", "normal", True)):
            if k_src in ins and os.path.exists(ins[k_src]):
                name = f"{key}_{k_dst}.png"
                d[k_dst] = copy_texture(ins[k_src], tex_dir, name, a.tex, normal)
        if "albedo" in d:
            d["color"] = [1.0, 1.0, 1.0]
        out["materials"][key] = d
        return key

    # ── 정적 메쉬: 책상 상판·보드·줄기·계란판(월드 좌표로 굽기) ──
    for path, mkey in (("/World/lab_environment/table/Cube/Mesh", "table_top"),
                       ("/World/lab_environment/whiteboard/board", "board"),
                       ("/World/egg_carton/shell", "egg_carton")):
        prim = st.GetPrimAtPath(path)
        geo, _ = mesh_unity(prim, world=True)
        if mkey in ("board", "egg_carton"):
            # 계란판: 옆면·컵 안쪽 일부 면의 감김·법선이 뒤집혀 있어 단면 렌더에서 투명하게 보였다(10-21 발견)
            # 보드 사각형의 앞면은 로봇 반대쪽(+y)을 향한다. Isaac(RTX)은 뒷면도 그리지만 Unity Standard 는 잘라 내므로 양면으로 만든다
            geo = double_sided(geo)
        out["meshes"][mkey] = geo
        mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        dc = UsdGeom.PrimvarsAPI(prim).GetPrimvar("displayColor")
        fb = dc.Get()[0] if dc and dc.HasValue() else None
        out["objects"].append({"name": mkey, "mesh": mkey, "material": add_material(mkey, mat, fb), "pos": [0, 0, 0], "rot": [0, 0, 0, 1], "scale": [1, 1, 1]})
    vine_done = False
    for prim in st.Traverse():
        pth = str(prim.GetPath())
        if pth.startswith("/World/vines/") and prim.IsA(UsdGeom.Mesh):
            if not vine_done:
                geo, _ = mesh_unity(prim, world=False)
                out["meshes"]["vine"] = geo; vine_done = True
                mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
                add_material("vine", mat, (0.1338, 0.1841, 0.0136))
            out["objects"].append({"name": pth.split("/")[3], "mesh": "vine", "material": "vine", **to_unity_trs(mat4(prim))})

    # ── 딸기: 메쉬 한 벌 + 인스턴스 12개 ──
    fruit_done = False
    for prim in st.Traverse():
        pth = str(prim.GetPath())
        m = re.match(r"^/World/(strawberry_(ripe|unripe)_\d+)/geo/fruit/mesh$", pth)
        if not m:
            continue
        if not fruit_done:
            geo, P = mesh_unity(prim, world=False)
            out["meshes"]["strawberry"] = geo; fruit_done = True
        kind = m.group(2)
        mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        mkey = add_material(f"strawberry_{kind}", mat)
        trs = to_unity_trs(mat4(prim))
        out["objects"].append({"name": m.group(1), "mesh": "strawberry", "material": mkey, "ripe": kind == "ripe", **trs})
        # 검사: 메쉬 월드 bbox 중심 vs CONTRACT 좌표
        M = mat4(prim); W = (M[:3, :3] @ P.T).T + M[:3, 3]
        ctr = (W.min(0) + W.max(0)) / 2
        name = m.group(1)
        if name in CONTRACT_RIPE:
            d = np.array(ctr) - np.array(CONTRACT_RIPE[name])
            out["checks"][name] = {"mesh_center_ros": r5(ctr), "contract": CONTRACT_RIPE[name], "diff_mm": r5(d * 1000)}

    # ── 상자(UsdGeom.Cube): 바닥·책상 다리·받침판·보드 가드 ──
    for prim in st.Traverse():
        if prim.GetTypeName() != "Cube":
            continue
        pth = str(prim.GetPath())
        if not pth.startswith("/World/lab_environment"):
            continue
        if UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible:
            out.setdefault("skipped_invisible", []).append(pth)   # 예: board_guard (Isaac 에서 보이지 않는 충돌 전용 상자)
            continue
        size = UsdGeom.Cube(prim).GetSizeAttr().Get()
        dc = UsdGeom.PrimvarsAPI(prim).GetPrimvar("displayColor")
        mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        key = "box_" + pth.split("/")[-1]
        add_material(key, mat, dc.Get()[0] if dc and dc.HasValue() else (0.7, 0.7, 0.7))
        trs = to_unity_trs(mat4(prim))
        trs["scale"] = r5(np.array(trs["scale"]) * size)
        out["boxes"].append({"name": pth.split("/")[-1], "material": key, **trs})

    # ── 보드 하이라이트 테두리 20장 (Isaac 은 visibility 로 켜고 끈다. 여기서는 전부 내보내고 Unity 가 켠다) ──
    hl_root = st.GetPrimAtPath("/World/lab_environment/whiteboard/highlight")
    out["highlights"] = []
    for prim in Usd.PrimRange(hl_root):
        if prim.IsA(UsdGeom.Mesh):
            name = prim.GetName()
            geo, _ = mesh_unity(prim, world=True)
            out["meshes"]["hl_" + name] = double_sided(geo)
            mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
            ins = shader_inputs(mat)
            out["highlights"].append({"name": name, "mesh": "hl_" + name})
            out["glow"] = {"color": color3(ins.get("emissive_color", (0.22, 0.48, 0.72))), "intensity": float(ins.get("emissive_intensity", 4.0))}

    # ── 보드 방위 글자 텍스처 9장 (tree_model.board_label_key 의 키) ──
    lab_dir = os.path.join(src, "strawberry_harvest/assets/props/textures/board_labels")
    out["board_label_textures"] = []
    for fn in sorted(os.listdir(lab_dir)):
        if fn.startswith("board_") and fn.endswith(".png"):
            key = fn[len("board_"):-4]
            t = copy_texture(os.path.join(lab_dir, fn), os.path.join(tex_dir, "board_labels"), fn, 2048)
            out["board_label_textures"].append({"key": key, "file": t["file"], "source_sha256": t["source_sha256"]})

    # ── 그리퍼의 D455 본체(보이는 조각) + 손목 컬러 카메라 — rh_p12_rn_base 기준 ──
    # 본체는 NVIDIA 자산이라 부품마다 경계 상자로만 낸다(bbox_box). 재질 값·카메라 자세·화각은 그대로.
    link = st.GetPrimAtPath("/World/robot_assembly/rh_p12_rn_base")
    ML = mat4(link)
    for prim in Usd.PrimRange(st.GetPrimAtPath("/World/robot_assembly/rh_p12_rn_base/rsd455")):
        if not prim.IsA(UsdGeom.Mesh) or UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible:
            continue
        name = "d455_" + prim.GetName()
        geo, _ = mesh_unity(prim, ref=ML)
        out["meshes"][name] = bbox_box(geo)
        mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        ins = shader_inputs(mat)
        out["materials"][name] = {"color": color3(ins.get("diffuse_color_constant", (0.55, 0.56, 0.58))),
                                  "smoothness": round(1.0 - float(ins.get("reflection_roughness_constant", 0.5)), 3),
                                  "metallic": float(min(0.6, ins.get("metallic_constant", 0.0)))}
        out["objects"].append({"name": name, "mesh": name, "material": name, "parent": "rh_p12_rn_base",
                               "pos": [0, 0, 0], "rot": [0, 0, 0, 1], "scale": [1, 1, 1]})
    camp = st.GetPrimAtPath("/World/robot_assembly/rh_p12_rn_base/rsd455/RSD455/Camera_OmniVision_OV9782_Color")
    cam = UsdGeom.Camera(camp)
    Mrel = np.linalg.inv(ML) @ mat4(camp)
    f, ha, va = cam.GetFocalLengthAttr().Get(), cam.GetHorizontalApertureAttr().Get(), cam.GetVerticalApertureAttr().Get()
    out["wrist_camera"] = {"parent": "rh_p12_rn_base", "pos": r5(C @ Mrel[:3, 3]), "forward": r5(C @ (-Mrel[:3, 2])),
                           "up": r5(C @ Mrel[:3, 1]), "vfov_deg": round(math.degrees(2 * math.atan(va / 2 / f)), 3),
                           "hfov_deg": round(math.degrees(2 * math.atan(ha / 2 / f)), 3), "res": [640, 480],
                           "source": str(camp.GetPath())}

    # ── HUD 라벨 PNG (Isaac 표시 스크립트가 쓰는 한글 라벨, Noto Sans CJK 로 미리 그린 것) ──
    hud_src = os.path.join(src, "strawberry_harvest/scripts/hud/labels")
    hud_dst = os.path.join(ROOT, "assets_src", "hud", "labels")
    os.makedirs(hud_dst, exist_ok=True)
    import shutil
    n_lab = 0
    for fn in sorted(os.listdir(hud_src)):
        if fn.endswith(".png") or fn == "manifest.json":
            shutil.copyfile(os.path.join(hud_src, fn), os.path.join(hud_dst, fn)); n_lab += 1
    out["source"]["hud_labels"] = {"dir": hud_src, "count": n_lab, "manifest_sha256": sha256(os.path.join(hud_src, "manifest.json"))}

    # ── 조명 ──
    for prim in st.Traverse():
        if "Light" not in prim.GetTypeName():
            continue
        ins = {at.GetName().replace("inputs:", ""): at.Get() for at in prim.GetAttributes() if at.GetName().startswith("inputs:")}
        M = mat4(prim)
        out["lights"].append({"name": prim.GetName(), "type": prim.GetTypeName(), "pos": r5(C @ M[:3, 3]),
                              "forward": r5(C @ (-M[:3, 2])), "color": color3(ins.get("color", (1, 1, 1))),
                              "intensity": float(ins.get("intensity", 1.0)), "width": ins.get("width"), "height": ins.get("height")})

    # ── 카메라 프리셋 (Kit 확장 설정 + 녹화 근접 구도) ──
    toml = os.path.join(src, "strawberry_harvest/kit_ext/strawberry.sim.setup/config/extension.toml")
    pres = toml_presets(toml)
    t, r, f, h = pres["long"]
    out["cameras"].append(camera_preset("isaac_long", t, r, f, h))
    rc = open(os.path.join(src, "strawberry_harvest/scripts/hud/record_camera.py"), encoding="utf-8").read()
    mclose = re.search(r"^CLOSE\s*=\s*\(([^)]*)\)", rc, re.M)
    if mclose:
        out["cameras"].append(camera_preset("isaac_record_close", [float(v) for v in mclose.group(1).split(",")], r, f, h))
    for p in ("side_s", "pair_a2", "pair_a3"):
        if pres[p][0]:
            out["cameras"].append(camera_preset(f"isaac_{p}", *pres[p]))
    out["source"]["camera_config"] = {"extension_toml": toml, "sha256": sha256(toml)}

    # ── 검사: 보드면 ──
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeWorldBound(
        st.GetPrimAtPath("/World/lab_environment/whiteboard/board")).ComputeAlignedRange()
    out["checks"]["board_plane_y_ros"] = round(bb.GetMin()[1], 4)

    # Unity JsonUtility 는 사전을 못 읽으므로 meshes·materials 를 이름 붙은 목록으로, checks 는 문자열로 바꿔 쓴다
    meshes_d, mats_d, checks_d = out["meshes"], out["materials"], out["checks"]
    out_u = dict(out)
    out_u["meshes"] = [{"name": k, **v} for k, v in meshes_d.items()]
    out_u["materials"] = []
    for k, v in mats_d.items():
        m = {"name": k, "color": v["color"], "smoothness": v["smoothness"], "metallic": v["metallic"],
             "albedo": v["albedo"]["file"] if "albedo" in v else "", "normal": v["normal"]["file"] if "normal" in v else ""}
        out_u["materials"].append(m)
    out_u["texture_sources"] = [{"file": v[t]["file"], "source": v[t]["source"], "source_sha256": v[t]["source_sha256"], "size": v[t]["size"]}
                                for v in mats_d.values() for t in ("albedo", "normal") if t in v]
    out_u["checks"] = json.dumps(checks_d, ensure_ascii=False)
    print(f"  보이지 않아 뺀 형상: {out.get('skipped_invisible', [])}")
    for o in out_u["objects"]:
        o.setdefault("ripe", False)
        o.setdefault("parent", "")
    for l in out_u["lights"]:
        l["width"] = float(l["width"] or 0); l["height"] = float(l["height"] or 0)
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "scene.json"), "w") as fjson:
        json.dump(out_u, fjson, separators=(",", ":"))
    # 요약
    print(f"[export_isaac_scene] 원천 {scene} (딸기 HEAD {head})")
    print(f"  메쉬 {len(out['meshes'])}: " + ", ".join(f"{k}({v['triangles']} tri)" for k, v in out["meshes"].items()))
    print(f"  오브젝트 {len(out['objects'])}, 상자 {len(out['boxes'])}, 재질 {len(out['materials'])}, 조명 {len(out['lights'])}, 카메라 {len(out['cameras'])}")
    worst = max(max(abs(x) for x in c["diff_mm"]) for k, c in out["checks"].items() if k.startswith("strawberry"))
    print(f"  딸기 메쉬 중심 − CONTRACT 좌표: 최대 {worst:.2f} mm")
    for k, c in out["checks"].items():
        if k.startswith("strawberry"):
            print(f"    {k}: {c['diff_mm']} mm")
    print(f"  보드면 y(ROS) = {out['checks']['board_plane_y_ros']} (기대 0.810)")
    print(f"  결과 {os.path.relpath(out_dir, ROOT)}/scene.json ({os.path.getsize(os.path.join(out_dir, 'scene.json')) / 1e6:.2f} MB), 텍스처 {len(os.listdir(tex_dir)) if os.path.isdir(tex_dir) else 0}개")
    return 0


if __name__ == "__main__":
    sys.exit(main())
