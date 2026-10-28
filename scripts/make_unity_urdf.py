#!/usr/bin/env python3
"""단계 3 — Unity 임포트용 URDF 사본을 만든다 (CONTRACT §1-4, §9-4 D-1·D-2).

원천은 딸기 리포 robot.urdf(Isaac 용 URDF)다. 원천은 읽기만 한다.
결과 폴더(Unity 배치 규칙: URDF 옆에 package:// 패키지명 폴더):
  assets_src/e0509/
    e0509.urdf
    dsr_description2/meshes/...
    rh_p12_rn_description/meshes/...
    strawberry_grasp_environment_description/meshes/...

사본에서 고치는 것은 아래뿐이다.
  1. 메쉬 절대 경로 → package://<패키지>/...
  2. 두 번째 <link name="world"> 와 world_fixed(→ rh_p12_rn_base) 삭제. 남기면 그리퍼 베이스의 부모가 둘이다
  3. rh_r2·rh_l2 upper 1.0 → 1.1 rad (Isaac 조립본과 같은 값, D-2)
  4. 기구와 무관한 ros2_control·transmission·gazebo 블록 삭제 (--keep-sim-blocks 로 끈다)
  5. (단계 4 추가) 두산 .dae 사본의 노드 <matrix> 를 정점·법선에 베이크해 단위행렬로 둔다 (bake_dae_node_transforms.py).
     Unity Collada 임포트가 노드 이동(mm, 전역→링크 프레임)을 정점과 다른 축으로 합성해 시각이 어긋나는 문제를 피한다.
     .stl 과 단위행렬 .dae 는 원본과 바이트 단위로 같다.

끝에 검증한다. 하나라도 틀리면 종료 코드 1.
  - 트리: 링크 15·관절 14, 부모가 둘인 링크 0, 루트 world
  - 원천 대비: 위 1–4 외에는 관절·링크 내용이 같다
  - 한계: Isaac 합성값(CONTRACT §2-4)과 같다
  - 메쉬: 전부 package://, 확장자 .dae/.stl/.obj, 베이크 안 한 복사본은 원본과 바이트 단위로 같고 베이크한 .dae 는 기록의
    sha256 전/후와 맞는다, 심볼릭 링크 없음
  - FK: 영점 rh_p12_rn_base z 1.123 m, TCP(+0.236 m) 영점 (0, 0, 1.359)·시작 자세 (0.0226, 0.2151, 0.6426) (CONTRACT §1-5·§2-6)

쓰는 법:
  python3 scripts/make_unity_urdf.py [--src ~/strawberry_grasp_environment] [--out assets_src/e0509]
"""
import argparse
import datetime
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bake_dae_node_transforms as bake_dae  # noqa: E402  (같은 scripts/ 폴더)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# package:// 이름 → 딸기 리포 src/ 아래 실제 폴더 (CONTRACT §1-5)
PACKAGES = [
    ("dsr_description2", "src/doosan_robot2/dsr_description2"),
    ("rh_p12_rn_description", "src/RH-P12-RN/rh_p12_rn_description"),
    ("strawberry_grasp_environment_description", "src/strawberry_grasp_environment_description"),
]
# 메쉬가 들어 있는 별도 git 저장소(딸기 리포의 .gitignore 대상, CONTRACT §1-1)
SUBREPOS = ["src/doosan_robot2", "src/RH-P12-RN"]
MESH_EXT = (".dae", ".stl", ".obj")
GRIPPER_UPPER_FIX = {"rh_r2": ("1.0", "1.1"), "rh_l2": ("1.0", "1.1")}

# Isaac 합성값(CONTRACT §2-4) — (lower, upper) deg, velocity deg/s
EXPECTED_LIMITS = {
    "joint_1": (-360.0, 360.0, 180.0),
    "joint_2": (-95.0, 95.0, 180.0),
    "joint_3": (-135.0, 135.0, 180.0),
    "joint_4": (-360.0, 360.0, 225.0),
    "joint_5": (-135.0, 135.0, 225.0),
    "joint_6": (-360.0, 360.0, 225.0),
    "rh_p12_rn": (0.0, 63.03, 372.4),
    "rh_r2": (0.0, 63.03, 372.4),
    "rh_l1": (0.0, 63.03, 372.4),
    "rh_l2": (0.0, 63.03, 372.4),
}
ARM = [f"joint_{i}" for i in range(1, 7)]
OVERVIEW_DEG = [87.98, -94.92, 129.89, 175.94, -31.34, 93.42]  # CONTRACT §3-6 시작 자세
TCP_OFFSET_M = 0.236                                            # CONTRACT §2-6


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def git(repo, *args):
    # --no-optional-locks: 조회만 해도 index 를 다시 쓰는 일을 막는다(딸기 리포는 읽기 전용)
    r = subprocess.run(["git", "--no-optional-locks", "-C", repo, *args],
                       capture_output=True, text=True, check=False)
    return r.stdout.strip()


def under(path, root):
    path, root = os.path.realpath(path), os.path.realpath(root)
    return path == root or path.startswith(root + os.sep)


# ── 변환 ────────────────────────────────────────────────────────────────

def transform(text, src_root, keep_sim_blocks):
    changes = []

    n_paths = 0
    for pkg, rel in PACKAGES:
        prefix = f'filename="{src_root}/{rel}/'
        n = text.count(prefix)
        text = text.replace(prefix, f'filename="package://{pkg}/')
        n_paths += n
    changes.append(f"메쉬 절대 경로 → package:// : {n_paths}곳")

    dup_world = re.compile(
        r"[ \t]*<!-- Used for fixing robot to Gazebo 'link 1' -->\n"
        r'[ \t]*<link name="world">\s*</link>\n'
        r'[ \t]*<joint name="world_fixed" type="fixed">\s*'
        r'<origin rpy="0 0 0" xyz="0 0 0"/>\s*'
        r'<parent link="world"/>\s*'
        r'<child link="rh_p12_rn_base"/>\s*'
        r"</joint>\n")
    text, n = dup_world.subn("", text)
    if n != 1:
        sys.exit(f"!! 중복 world·world_fixed 블록을 정확히 1곳에서 찾지 못함(찾은 수 {n})")
    changes.append("중복 world 링크 + world_fixed(→ rh_p12_rn_base) 삭제: 1곳")

    for joint, (old, new) in GRIPPER_UPPER_FIX.items():
        pat = re.compile(r'(<joint name="%s" type="revolute">(?:(?!</joint>).)*?<limit [^>]*?upper=")%s(")'
                         % (re.escape(joint), re.escape(old)), re.S)
        text, n = pat.subn(r"\g<1>%s\g<2>" % new, text)
        if n != 1:
            sys.exit(f"!! {joint} 의 upper=\"{old}\" 를 정확히 1곳에서 찾지 못함(찾은 수 {n})")
        changes.append(f"{joint} upper {old} → {new} rad")

    if not keep_sim_blocks:
        for tag in ("ros2_control", "transmission", "gazebo"):
            pat = re.compile(r"[ \t]*<%s\b[^>]*>.*?</%s>[ \t]*\n" % (tag, tag), re.S)
            text, n = pat.subn("", text)
            changes.append(f"{tag} 블록 삭제: {n}개")
    return text, changes


def header(src_urdf, prov, changes):
    lines = [
        "<!-- e0509 Unity 시뮬레이터용 URDF. scripts/make_unity_urdf.py 가 만든다. 손으로 고치지 말 것. -->",
        f"<!-- 원천: ~/{os.path.relpath(src_urdf, os.path.expanduser('~'))} (Isaac 용 URDF) -->",
        f"<!--   딸기 리포 HEAD {prov['head']} · robot.urdf 마지막 커밋 {prov['urdf_commit']} · sha256 {prov['urdf_sha256']} -->",
    ]
    for rel, h in prov["subrepos"].items():
        lines.append(f"<!--   메쉬 저장소 {rel} @ {h} -->")
    lines.append("<!-- 원천 대비 바꾼 것: " + " / ".join(changes) + " -->")
    return "\n".join(lines) + "\n"


def replace_xacro_header(text, hdr):
    # xacro 가 붙인 자동 생성 경고 주석을 출처 주석으로 바꾼다
    m = re.match(r'(<\?xml[^>]*\?>\n)((?:<!--[^\n]*-->\n)+)', text)
    if not m:
        sys.exit("!! 원천 URDF 머리 주석 모양이 예상과 다름")
    return m.group(1) + hdr + text[m.end():]


# ── 검증 ────────────────────────────────────────────────────────────────

class Checks:
    def __init__(self):
        self.results = []

    def __call__(self, name, ok, detail=""):
        self.results.append((name, bool(ok)))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))

    @property
    def failed(self):
        return [n for n, ok in self.results if not ok]


def canon(e, path_map=None, upper_back=None):
    """요소를 비교 가능한 튜플로. 사본에서 바꾼 값은 원천 표기로 되돌린다
    (메쉬 filename 은 path_map 으로, limit upper 는 upper_back 으로)."""
    attrs = dict(e.attrib)
    if path_map and e.tag == "mesh" and "filename" in attrs:
        attrs["filename"] = path_map(attrs["filename"])
    if upper_back is not None and e.tag == "limit":
        attrs["upper"] = upper_back
    text = (e.text or "").strip()
    return (e.tag, tuple(sorted(attrs.items())), text,
            tuple(canon(c, path_map, upper_back) for c in e if c.tag is not ET.Comment))


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    # URDF: R = Rz(y) · Ry(p) · Rx(r)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def axis_angle(ax, q):
    x, y, z = ax
    n = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / n, y / n, z / n
    c, s, t = math.cos(q), math.sin(q), 1 - math.cos(q)
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]


def homog(R, p):
    return [R[0] + [p[0]], R[1] + [p[1]], R[2] + [p[2]], [0.0, 0.0, 0.0, 1.0]]


def mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def floats(s, default):
    return [float(v) for v in s.split()] if s else default


def fk(joints_by_child, link, q):
    """world → link 4x4 변환. q: 관절 이름 → rad."""
    T = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    chain = []
    while link in joints_by_child:
        j = joints_by_child[link]
        chain.append(j)
        link = j.find("parent").get("link")
    for j in reversed(chain):
        o = j.find("origin")
        xyz = floats(o.get("xyz") if o is not None else None, [0.0, 0.0, 0.0])
        rpy = floats(o.get("rpy") if o is not None else None, [0.0, 0.0, 0.0])
        T = mat_mul(T, homog(rpy_matrix(*rpy), xyz))
        if j.get("type") in ("revolute", "continuous"):
            ax = j.find("axis")
            T = mat_mul(T, homog(axis_angle(floats(ax.get("xyz") if ax is not None else None, [1.0, 0.0, 0.0]),
                                            q.get(j.get("name"), 0.0)), [0.0, 0.0, 0.0]))
    return T


def tcp(joints_by_child, q):
    T = fk(joints_by_child, "rh_p12_rn_base", q)
    return [T[i][3] + T[i][2] * TCP_OFFSET_M for i in range(3)], T[2][3]


def verify(src_urdf, out_urdf, out_dir, src_root, bake_records=None):
    c = Checks()
    src = ET.parse(src_urdf).getroot()
    out = ET.parse(out_urdf).getroot()
    out_text = open(out_urdf, encoding="utf-8").read()

    # 트리
    links = [l.get("name") for l in out.findall("link")]
    joints = out.findall("joint")
    children = [j.find("child").get("link") for j in joints]
    roots = sorted(set(links) - set(children))
    c("트리: 링크 15·관절 14", len(links) == 15 and len(joints) == 14, f"링크 {len(links)}·관절 {len(joints)}")
    c("트리: 링크 이름 중복 없음", len(links) == len(set(links)))
    c("트리: 부모가 둘인 링크 없음", len(children) == len(set(children)))
    c("트리: 루트 하나 = world", roots == ["world"], f"루트 {roots}")
    c("ros2_control·transmission·gazebo 없음",
      not any(out.findall(t) for t in ("ros2_control", "transmission", "gazebo")))

    # 원천 대비 관절
    def src_joint(name, child):
        cands = [j for j in src.findall("joint") if j.get("name") == name and j.find("child").get("link") == child]
        return cands[0] if len(cands) == 1 else None

    joint_diffs = []
    for j in joints:
        name, child = j.get("name"), j.find("child").get("link")
        sj = src_joint(name, child)
        if sj is None:
            joint_diffs.append(f"{name}: 원천에 없음")
            continue
        upper_back = None
        if name in GRIPPER_UPPER_FIX:
            old, new = GRIPPER_UPPER_FIX[name]
            got = (sj.find("limit").get("upper"), j.find("limit").get("upper"))
            if got != (old, new):
                joint_diffs.append(f"{name}: upper {got[0]} → {got[1]} (기대 {old} → {new})")
            upper_back = old
        if canon(sj) != canon(j, upper_back=upper_back):
            joint_diffs.append(f"{name}: 내용이 다름")
    removed = [j.get("name") + "→" + j.find("child").get("link") for j in src.findall("joint")
               if not any(o.get("name") == j.get("name") and o.find("child").get("link") == j.find("child").get("link")
                          for o in joints)]
    c("원천 대비 관절: 그리퍼 상한 2곳 말고는 같음", not joint_diffs, "; ".join(joint_diffs) or "14개 비교")
    c("원천 대비 관절: 빠진 것은 world_fixed→rh_p12_rn_base 하나", removed == ["world_fixed→rh_p12_rn_base"],
      f"빠진 것 {removed}")

    # 원천 대비 링크 (메쉬 경로는 원천 표기로 되돌려 비교)
    def back_to_src(fn):
        for pkg, rel in PACKAGES:
            if fn.startswith(f"package://{pkg}/"):
                return f"{src_root}/{rel}/" + fn[len(f"package://{pkg}/"):]
        return fn

    link_diffs = []
    for l in out.findall("link"):
        cands = [s for s in src.findall("link") if s.get("name") == l.get("name")]
        if not any(canon(s) == canon(l, back_to_src) for s in cands):
            link_diffs.append(l.get("name"))
    c("원천 대비 링크: 관성·visual·collision 같음(메쉬 경로 표기만 다름)", not link_diffs,
      ", ".join(link_diffs) or "15개 비교")

    mats_src = sorted(canon(m) for m in src.findall("material"))
    mats_out = sorted(canon(m) for m in out.findall("material"))
    c("원천 대비 최상위 material 같음", mats_src == mats_out, f"{len(mats_out)}개")

    # 한계
    lim_rows, lim_bad = [], []
    for j in joints:
        name = j.get("name")
        if name not in EXPECTED_LIMITS:
            continue
        lim = j.find("limit")
        lo, hi, vel = (math.degrees(float(lim.get(k))) for k in ("lower", "upper", "velocity"))
        elo, ehi, evel = EXPECTED_LIMITS[name]
        ok = abs(lo - elo) < 0.01 and abs(hi - ehi) < 0.01 and abs(vel - evel) < 0.1
        lim_rows.append(f"{name} [{lo:.2f}, {hi:.2f}]° {vel:.1f}°/s")
        if not ok:
            lim_bad.append(name)
    c("한계 = Isaac 합성값 (CONTRACT §2-4)", not lim_bad and len(lim_rows) == 10,
      ", ".join(lim_bad) if lim_bad else f"{len(lim_rows)}개 관절")
    for row in lim_rows:
        print(f"         {row}")

    # 메쉬
    fns = [m.get("filename") for m in out.iter("mesh")]
    bad_scheme = [f for f in fns if not f.startswith("package://")]
    c("메쉬 참조 전부 package://", not bad_scheme and fns, f"{len(fns)}곳" if not bad_scheme else bad_scheme[0])
    c("URDF 속성에 절대 경로·file:// 없음", not re.search(r'="(/|file://)', out_text))
    uniq = sorted(set(fns))
    bad_ext = [f for f in uniq if not f.lower().endswith(MESH_EXT)]
    c("메쉬 확장자 .dae/.stl/.obj 만", not bad_ext, ", ".join(bad_ext) or f"{len(uniq)}개 파일")
    missing, differ, links_found, baked_ok, baked_bad = [], [], [], [], []
    # 단계 4: .dae 는 노드 행렬을 정점에 베이크한 사본이다(bake_dae_node_transforms.py). 기록의 sha256 전/후와 맞아야 한다.
    bake_by_file = {os.path.realpath(r["file"]): r for r in (bake_records or [])}
    for f in uniq:
        rel = f[len("package://"):]
        dst = os.path.join(out_dir, rel)
        src_file = back_to_src(f)
        if not os.path.isfile(dst):
            missing.append(rel)
            continue
        if os.path.islink(dst):
            links_found.append(rel)
        h_dst, h_src = sha256(dst), sha256(src_file)
        rec = bake_by_file.get(os.path.realpath(dst))
        if rec is not None and rec["changed"]:
            # 베이크된 파일: 원본 해시가 기록의 before 와, 현재 해시가 기록의 after 와 같아야 한다
            (baked_ok if (rec["sha256_before"] == h_src and rec["sha256_after"] == h_dst) else baked_bad).append(rel)
        elif h_dst != h_src:
            differ.append(rel)
    c("메쉬 파일 전부 있음(URDF 옆 패키지 폴더)", not missing, ", ".join(missing))
    c("베이크 안 한 메쉬 복사본 = 원본 (sha256)", not differ, ", ".join(differ))
    c(".dae 노드 행렬 베이크 기록과 일치 (원본 sha256 = before, 사본 = after)", not baked_bad,
      ", ".join(baked_bad) if baked_bad else f"{len(baked_ok)}개 베이크, 나머지는 단위행렬이라 원본 그대로")
    c("심볼릭 링크 없음", not links_found and not any(
        os.path.islink(os.path.join(dp, n)) for dp, ds, fs in os.walk(out_dir) for n in ds + fs))
    on_disk = sorted(os.path.relpath(os.path.join(dp, n), out_dir)
                     for dp, _, fs in os.walk(out_dir) for n in fs)
    expected_files = sorted([os.path.basename(out_urdf)] + [f[len("package://"):] for f in uniq])
    c("결과 폴더에 다른 파일 없음", on_disk == expected_files, f"{len(on_disk)}개 파일")

    # FK
    by_child = {j.find("child").get("link"): j for j in joints}
    zero = {n: 0.0 for n in ARM}
    tcp0, flange_z = tcp(by_child, zero)
    over = {n: math.radians(v) for n, v in zip(ARM, OVERVIEW_DEG)}
    tcp1, _ = tcp(by_child, over)

    def close(a, b, tol):
        return all(abs(x - y) <= tol for x, y in zip(a, b))

    c("FK 영점: rh_p12_rn_base z = 1.123 m", abs(flange_z - 1.123) < 1e-6, f"z {flange_z:.6f}")
    # 관절 원점 rpy 가 π/2 가 아니라 1.5708 로 적혀 있어 x·y 에 수 µm 가 남는다(원본 기하 그대로)
    c("FK 영점: TCP = (0, 0, 1.359) ±10 µm", close(tcp0, [0.0, 0.0, 1.359], 1e-5),
      "(" + ", ".join(f"{v:.4f}" for v in tcp0) + ")")
    c("FK 시작 자세: TCP ≈ (0.0226, 0.2151, 0.6426) ±0.5 mm", close(tcp1, [0.0226, 0.2151, 0.6426], 5e-4),
      "(" + ", ".join(f"{v:.4f}" for v in tcp1) + ")")

    r = subprocess.run(["xmllint", "--noout", out_urdf], capture_output=True, text=True)
    c("xmllint 문법 검사", r.returncode == 0, r.stderr.strip()[:200])
    return c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=os.environ.get("STRAWBERRY_WS", os.path.expanduser("~/strawberry_grasp_environment")))
    ap.add_argument("--out", default=os.path.join(ROOT, "assets_src", "e0509"))
    ap.add_argument("--keep-sim-blocks", action="store_true", help="ros2_control·transmission·gazebo 블록을 남긴다")
    args = ap.parse_args()

    src_root = os.path.realpath(os.path.expanduser(args.src))
    out_dir = os.path.realpath(args.out)
    src_urdf = os.path.join(src_root, "robot.urdf")
    out_urdf = os.path.join(out_dir, "e0509.urdf")
    if under(out_dir, src_root):
        sys.exit(f"!! 결과 폴더가 딸기 리포 안이다: {out_dir}")
    if not under(out_dir, ROOT) or out_dir == ROOT:
        sys.exit(f"!! 결과 폴더는 이 프로젝트 안의 하위 폴더여야 한다: {out_dir}")

    sha_before = sha256(src_urdf)
    prov = {
        "head": git(src_root, "rev-parse", "--short", "HEAD"),
        "urdf_commit": git(src_root, "log", "-1", "--format=%h", "--", "robot.urdf"),
        "urdf_dirty": bool(git(src_root, "status", "--porcelain", "--", "robot.urdf")),
        "urdf_sha256": sha_before,
        "subrepos": {rel: git(os.path.join(src_root, rel), "rev-parse", "--short", "HEAD") for rel in SUBREPOS},
    }
    print(f"[make_unity_urdf] {datetime.datetime.now():%Y-%m-%d %H:%M}")
    print(f"  원천 {src_urdf}")
    print(f"    딸기 리포 HEAD {prov['head']} · robot.urdf 마지막 커밋 {prov['urdf_commit']}"
          f" · 작업 트리 변경 {'있음' if prov['urdf_dirty'] else '없음'}")
    print(f"    sha256 {sha_before}")
    for rel, h in prov["subrepos"].items():
        print(f"    메쉬 저장소 {rel} @ {h}")
    print(f"  결과 {out_dir}")

    text = open(src_urdf, encoding="utf-8").read()
    text, changes = transform(text, src_root, args.keep_sim_blocks)
    text = replace_xacro_header(text, header(src_urdf, prov, changes))
    for ch in changes:
        print(f"  변경: {ch}")

    if os.path.exists(out_dir):
        shutil.rmtree(out_dir)
    os.makedirs(out_dir)
    with open(out_urdf, "w", encoding="utf-8") as f:
        f.write(text)
    for fn in sorted(set(re.findall(r'filename="package://([^"]+)"', text))):
        pkg, rest = fn.split("/", 1)
        rel = dict(PACKAGES)[pkg]
        src_file = os.path.join(src_root, rel, rest)
        if os.path.islink(src_file) or not under(src_file, src_root):
            sys.exit(f"!! 원본 메쉬가 심볼릭 링크이거나 딸기 리포 밖을 가리킨다: {src_file}")
        dst = os.path.join(out_dir, fn)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copyfile(src_file, dst)

    # 단계 4에서 추가: 두산 .dae 의 노드 <matrix>(전역 프레임 정점 → 링크 프레임 이동, mm)를 정점·법선에 베이크한다.
    # Unity Collada 임포트 + URDF-Importer 후처리가 노드 이동을 정점과 다른 축으로 합성해 링크 시각이 최대 1.1 m 어긋나는
    # 문제를 피한다(logs/step4/04·05_build_scene*.extract.log). 결과 정점 집합은 Isaac(assimp)이 보는 것과 같다.
    print("베이크 (.dae 노드 행렬 → 정점·법선)")
    bake_records = bake_dae.bake_dir(Path(out_dir) / "dsr_description2" / "meshes", log=print)
    bake_json = os.path.join(ROOT, "logs", "step4", "bake_dae.json")
    os.makedirs(os.path.dirname(bake_json), exist_ok=True)
    with open(bake_json, "w") as f:
        json.dump(bake_records, f, indent=1, ensure_ascii=False)
    print(f"  베이크 {sum(1 for r in bake_records if r['changed'])}/{len(bake_records)}개, 기록 {os.path.relpath(bake_json, ROOT)}")

    print("검증")
    c = verify(src_urdf, out_urdf, out_dir, src_root, bake_records)
    c("원천 robot.urdf 그대로 (sha256 전후 같음)", sha256(src_urdf) == sha_before)
    if c.failed:
        print(f"[make_unity_urdf] 실패 {len(c.failed)}건")
        return 1
    print(f"[make_unity_urdf] 전부 통과 ({len(c.results)}건)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
