#!/usr/bin/env python3
"""Collada(.dae) 노드 변환 베이크 — Unity 임포트용 사본 전처리 (단계 4, CONTRACT §1-5·§9-3).

두산 e0509 메쉬(.dae, Blender 3.4.1 내보내기)는 정점이 로봇 **전역 영점 프레임**(mm)에 있고, 씬 노드의 `<matrix>` 가
링크 프레임으로 되돌린다(예: E0509_6_1 정점 z 1055–1123, 노드 이동 (0, 0, −1123); E0509_3_2 는 X축 90° 회전).
Isaac(assimp)은 이 행렬을 정점에 적용해서 맞게 놓지만, Unity 의 Collada 임포트 + URDF-Importer 의 ColladaAssetPostProcessor
조합은 노드 이동을 정점과 다른 축으로 합성해 링크 시각이 최대 1.1 m 어긋났다(단계 4 BuildScene 로그 04/05).

이 스크립트는 사본의 각 노드 행렬 M 을 그 노드가 인스턴스하는 geometry 의 POSITION(점: M·p)·NORMAL(법선: R·n, 정규화)에
직접 적용하고 `<matrix>` 를 단위행렬로 바꾼다. 결과 정점 집합은 Isaac 이 보는 것과 같다. 단위행렬인 파일은 건드리지 않는다.
바꾼 파일에는 XML 선언 바로 아래에 수정 고지 주석(NOTICE)을 넣는다(Apache 2.0 §4(b), THIRD_PARTY.md §2-3).
딸기 리포 원본은 읽지도 않는다(사본만 고친다).

쓰는 법:
  python3 -B scripts/bake_dae_node_transforms.py [assets_src/e0509/dsr_description2/meshes] [--check] [--json logs/step4/bake_dae.json]
  --check : 바꾸지 않고 베이크가 필요한 파일만 보고 (종료 코드 1 이면 필요)
검증: 베이크 뒤 각 geometry 의 bbox 8 꼭짓점이 M·(원래 bbox 꼭짓점) 과 1e-3 mm 안에서 같아야 한다. 아니면 종료 코드 2.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

NS = "http://www.collada.org/2005/11/COLLADASchema"
C = "{%s}" % NS
ET.register_namespace("", NS)      # 저장할 때 ns0: 접두어가 붙지 않게

# 바꾼 파일에 넣는 수정 고지(Apache 2.0 §4(b)). {rel} = meshes/ 아래 상대 경로. XML 주석이라 "--" 를 쓰지 않는다.
NOTICE = """<!--
  NOTICE: This file was modified by OceanShape for e0509_unity_sim.
  Original: DoosanRobotics/doosan-robot2, dsr_description2/meshes/{rel}
  Change: each node <matrix> was applied to the POSITION and NORMAL arrays of its geometry
  and the node matrix was set to identity (scripts/bake_dae_node_transforms.py), because the
  Unity Collada importer composed the node translation on a different axis than the vertices.
  Upstream license: BSD 3-Clause (repository LICENSE, third_party_licenses/doosan-robot2_BSD-3-Clause.txt)
  and Apache License 2.0 (dsr_description2/package.xml). See THIRD_PARTY.md.
-->"""


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def read_floats(el) -> np.ndarray:
    return np.array([float(x) for x in el.text.split()], dtype=np.float64)


def write_floats(el, arr: np.ndarray, per_line=3):
    # Blender 내보내기와 비슷한 형식: 공백 구분, 유효숫자 넉넉히(원본은 7자리). 소수 자릿수 때문에 재베이크해도 값이 드리프트하지 않게 9자리.
    flat = arr.reshape(-1)
    el.text = " ".join(f"{v:.9g}" for v in flat)
    el.set("count", str(flat.size))


def geometry_sources(geom):
    """geometry → {'POSITION': float_array el, 'NORMAL': float_array el} (triangles/polylist 입력과 <vertices> 를 따라간다)."""
    mesh = geom.find(C + "mesh")
    if mesh is None:
        return {}
    sources = {s.attrib["id"]: s for s in mesh.findall(C + "source")}
    vertices = {v.attrib["id"]: v for v in mesh.findall(C + "vertices")}
    out = {}

    def resolve(semantic, url):
        sid = url.lstrip("#")
        if sid in vertices:                          # <input semantic="VERTEX" source="#id-vertices"> → 안의 POSITION 등
            for inp in vertices[sid].findall(C + "input"):
                resolve(inp.attrib["semantic"], inp.attrib["source"])
            return
        if sid in sources and semantic in ("POSITION", "NORMAL"):
            fa = sources[sid].find(C + "float_array")
            if fa is not None:
                out[semantic] = fa

    for prim in mesh:
        if prim.tag in (C + "triangles", C + "polylist", C + "polygons", C + "lines", C + "linestrips", C + "tristrips", C + "trifans"):
            for inp in prim.findall(C + "input"):
                resolve(inp.attrib["semantic"], inp.attrib["source"])
    return out


def bbox_corners(pts: np.ndarray) -> np.ndarray:
    lo, hi = pts.min(0), pts.max(0)
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def bake_file(path: Path, check_only=False, rel=None):
    """한 파일을 베이크. 반환: 기록 dict (changed=False 면 단위행렬이라 건드리지 않음). rel = 고지에 적을 meshes/ 아래 경로."""
    rec = {"file": str(path), "sha256_before": sha256(path), "nodes": [], "changed": False}
    tree = ET.parse(path)
    root = tree.getroot()
    geoms = {g.attrib["id"]: g for g in root.iter(C + "geometry")}

    # 같은 geometry 를 두 노드가 인스턴스하면 한 번만 베이크할 수 있다 → 지원하지 않고 멈춘다.
    used = {}
    for node in root.iter(C + "node"):
        for ig in node.findall(C + "instance_geometry"):
            gid = ig.attrib["url"].lstrip("#")
            used.setdefault(gid, []).append(node.attrib.get("id", "?"))
    shared = {g: n for g, n in used.items() if len(n) > 1}
    if shared:
        raise SystemExit(f"{path}: geometry 가 여러 노드에서 인스턴스됨 {shared} — 지원하지 않음")

    for node in root.iter(C + "node"):
        mats = node.findall(C + "matrix")
        others = [k for k in node if k.tag in (C + "translate", C + "rotate", C + "scale", C + "lookat")]
        if others:
            raise SystemExit(f"{path}: <matrix> 가 아닌 변환 요소 {[o.tag for o in others]} — 지원하지 않음")
        if node.find(C + "node") is not None:
            raise SystemExit(f"{path}: 중첩 노드 — 지원하지 않음")
        if len(mats) != 1:
            if mats:
                raise SystemExit(f"{path}: 노드 {node.attrib.get('id')} 에 <matrix> 가 {len(mats)}개")
            continue
        M = read_floats(mats[0]).reshape(4, 4)
        if np.allclose(M, np.eye(4), atol=1e-9):
            rec["nodes"].append({"node": node.attrib.get("id"), "identity": True})
            continue
        R, t = M[:3, :3], M[:3, 3]
        if not (np.allclose(R @ R.T, np.eye(3), atol=1e-6) and np.isclose(np.linalg.det(R), 1.0, atol=1e-6)):
            raise SystemExit(f"{path}: 노드 {node.attrib.get('id')} 의 회전부가 직교 회전이 아님 (스케일/반전) — 지원하지 않음")
        nrec = {"node": node.attrib.get("id"), "identity": False, "translate": t.tolist(), "rotation": R.tolist(), "geometries": []}
        for ig in node.findall(C + "instance_geometry"):
            gid = ig.attrib["url"].lstrip("#")
            geom = geoms[gid]
            srcs = geometry_sources(geom)
            if "POSITION" not in srcs:
                raise SystemExit(f"{path}: geometry {gid} 에 POSITION 소스 없음")
            P = read_floats(srcs["POSITION"]).reshape(-1, 3)
            before = bbox_corners(P)
            P2 = P @ R.T + t
            grec = {"geometry": gid, "vertices": int(P.shape[0]),
                    "bbox_before": [P.min(0).tolist(), P.max(0).tolist()],
                    "bbox_after": [P2.min(0).tolist(), P2.max(0).tolist()]}
            if not check_only:
                write_floats(srcs["POSITION"], P2)
                if "NORMAL" in srcs:
                    N = read_floats(srcs["NORMAL"]).reshape(-1, 3)
                    N2 = N @ R.T
                    norms = np.linalg.norm(N2, axis=1, keepdims=True)
                    N2 = np.where(norms > 0, N2 / np.where(norms == 0, 1, norms), N2)
                    write_floats(srcs["NORMAL"], N2)
                    grec["normals"] = int(N.shape[0])
                # 검증: 베이크된 점의 bbox 꼭짓점 집합 == M·(원래 bbox 꼭짓점) 집합 (회전이 축 정렬일 때 성립. 아니면 점 전체로 비교)
                expect = before @ R.T + t
                if np.allclose(R, np.round(R), atol=1e-6):
                    got = bbox_corners(P2)
                    ok = np.allclose(np.sort(got, axis=0), np.sort(expect, axis=0), atol=1e-3)
                else:
                    ok = np.allclose(P2, P @ R.T + t, atol=1e-6)
                grec["verified"] = bool(ok)
                if not ok:
                    raise SystemExit(f"{path}: {gid} 베이크 검증 실패")
            nrec["geometries"].append(grec)
        if not check_only:
            mats[0].text = " ".join(f"{v:g}" for v in np.eye(4).reshape(-1))
            rec["changed"] = True
        else:
            rec["changed"] = True        # 필요함을 뜻함
        rec["nodes"].append(nrec)

    if rec["changed"] and not check_only:
        tree.write(path, encoding="utf-8", xml_declaration=True)
        text = path.read_text(encoding="utf-8")
        decl_end = text.index("?>") + 2           # ElementTree 가 쓴 XML 선언 바로 뒤
        notice = NOTICE.format(rel=rel or path.name)
        assert "--" not in notice[4:-3]
        path.write_text(text[:decl_end] + "\n" + notice + text[decl_end:], encoding="utf-8")
        rec["notice"] = True
        rec["sha256_after"] = sha256(path)
    else:
        rec["sha256_after"] = rec["sha256_before"]
    return rec


def bake_dir(mesh_dir: Path, check_only=False, log=print):
    files = sorted(mesh_dir.rglob("*.dae"))
    if not files:
        raise SystemExit(f".dae 없음: {mesh_dir}")
    records = []
    for p in files:
        rec = bake_file(p, check_only, rel=p.relative_to(mesh_dir).as_posix())
        records.append(rec)
        tag = ("필요" if check_only else "베이크") if rec["changed"] else "단위행렬·그대로"
        nz = [n for n in rec["nodes"] if not n.get("identity")]
        detail = "; ".join(f"{n['node']}: t={np.round(n['translate'], 1).tolist()}{' R' if not np.allclose(n['rotation'], np.eye(3)) else ''}" for n in nz)
        log(f"  {tag:10s} {p.relative_to(mesh_dir)}  {detail}")
    return records


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mesh_dir", nargs="?", default="assets_src/e0509/dsr_description2/meshes")
    ap.add_argument("--check", action="store_true", help="바꾸지 않고 베이크가 필요한 파일만 보고")
    ap.add_argument("--json", default=None, help="기록을 JSON 으로 저장")
    args = ap.parse_args(argv)
    mesh_dir = Path(args.mesh_dir)
    print(f"[bake_dae] {mesh_dir} ({'check' if args.check else 'bake'})")
    records = bake_dir(mesh_dir, args.check)
    changed = [r for r in records if r["changed"]]
    print(f"[bake_dae] 파일 {len(records)}개, {'베이크 필요' if args.check else '베이크'} {len(changed)}개")
    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        with open(args.json, "w") as f:
            json.dump(records, f, indent=1, ensure_ascii=False)
        print(f"[bake_dae] 기록 → {args.json}")
    if args.check and changed:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
