#!/usr/bin/env python3
"""Isaac HUD 와 같은 상태를 Unity 에 넘기는 중계 (단계 8 화면). 딸기 리포 HUD 모듈을 읽기만 해서 그대로 쓴다.

Isaac 에서는 노드 4개의 계측(harvest_probe)이 /tmp/harvest_hud_<role>.json 을 0.1 초마다 쓰고, Kit 안의
isaac_sim_viewport_display.py 가 bus_merge·tree_model·result_bar·status_bus 로 합쳐 그린다. 이 중계는 같은
모듈로 같은 계산을 해서 '무엇을 그릴지'(뷰 모델)만 E0509_HUD_VIEW(기본 /tmp/e0509_hud_view.json)에 쓴다.
Unity 의 HudOverlay·BoardDisplay·WristCameraInset 이 그 파일을 읽어 그리기만 한다 — 규칙의 단일 출처는 딸기 리포 모듈이다.
isaac_sim_viewport_display.py 는 omni 를 import 해서 여기서 쓸 수 없으므로, 그 안의 작은 함수 셋(harvest_key,
highlight_key·HIGHLIGHT_ON, PHASE_COLOR)과 패널 치수는 아래에 옮겨 적었다(원본 줄 번호를 단다).

딸기 리포에 아무것도 쓰지 않는다: import 만 하고(-B, PYTHONDONTWRITEBYTECODE), status_bus 의 쓰기 API(publish·bump·
append·heartbeat)는 부르지 않는다. 읽는 파일은 /tmp/harvest_hud_*.json, 쓰는 파일은 /tmp 의 뷰 모델 하나.
  python3 -B scripts/hud_relay.py [--out /tmp/e0509_hud_view.json] [--targets 8]   (--targets = 결과 막대 칸 수 = 씬의 익은 과실 수)
"""
import argparse
import json
import os
import signal
import sys
import time

HUD_DIR = os.path.expanduser(os.environ.get(
    "HARVEST_HUD_DIR", "~/strawberry_grasp_environment/strawberry_harvest/scripts/hud"))
sys.dont_write_bytecode = True
sys.path.insert(0, HUD_DIR)
import bus_merge  # noqa: E402
import result_bar  # noqa: E402
import status_bus  # noqa: E402
import tree_model  # noqa: E402

# ── isaac_sim_viewport_display.py 에서 옮긴 값 (딸기 HEAD 2797bb5) ──
PANEL_WIDTH, PAD, ROW_GAP, HEAD_W = 440, 16, 12, 60              # :152,156-158
TREE_W = PANEL_WIDTH - 2 * PAD - HEAD_W                          # _row_tree: PANEL_WIDTH - 2*PAD - HEAD_W
PHASE_COLOR = {                                                   # :199-207
    "IDLE": 0x7B8494, "SCAN_MOVE": 0x4A9EFF, "DETECT": 0x00C8C8, "PLAN": 0xA78BFA,
    "APPROACH": 0xFFB547, "ENTER": 0xFFB547, "GRASP": 0xFF7A3D, "DETACH": 0xFF3D5C,
    "RETREAT": 0x4A9EFF, "PLACE": 0x5AD469, "RETURN": 0x4A9EFF, "DONE": 0x5AD469,
}
BAR_STATES = [s for s in status_bus.SEQUENCE_STATES if s != "IDLE"]   # :230 (11 칸)
HIGHLIGHT_QUADS = ("nw", "ne", "sw", "se")                       # :276-280
HIGHLIGHT_ON = {"home": set(HIGHLIGHT_QUADS), "nw": {"nw"}, "ne": {"ne"}, "sw": {"sw"}, "se": {"se"}}
HIGHLIGHT_ON.update({"%s/%s" % (q, s): {"%s_%s" % (q, s)} for q in HIGHLIGHT_QUADS for s in HIGHLIGHT_QUADS})


def harvest_key(snap):                                           # :241-261
    if snap["result"]["finished"]:
        return "done"
    run = snap.get("run")
    if not isinstance(run, dict) or not run.get("started_at"):
        return "idle"
    return "stopped" if run.get("ended_at") else "running"


def highlight_key(region, tree):                                 # :282-288
    try:
        cur = str((tree or {}).get("current") or "")
    except Exception:
        cur = ""
    return cur if cur in HIGHLIGHT_ON and "/" in cur else region


def rgba(t):
    return [int(v) for v in t]


def hexrgba(h, a=255):
    return [(h >> 16) & 255, (h >> 8) & 255, h & 255, a]


def tree_draw(tree, lay):
    """tree_model.view + node_paint → Unity 가 그릴 목록(트리 영역 좌표, Kit 논리 픽셀)."""
    v = tree_model.view(tree, lay)
    out = []

    def node(item, y, h, key, nv, name, size, center=False, tag=False):
        p = tree_model.node_paint(nv["style"], nv["border"], bool(nv.get("dim")))
        out.append({"k": "rect", "x": item["x"], "y": y, "w": item["w"], "h": h, "fill": rgba(p["fill"]),
                    "border": rgba(p["border"]), "bw": p["bw"], "r": 6})
        if center:
            out.append({"k": "text", "x": item["x"], "y": y, "w": item["w"], "h": h, "t": name, "s": size,
                        "c": rgba(p["name"]), "a": "c"})
            return
        row_y = y + (tree_model.L1_PAD_TOP if tag else 0)
        row_h = tree_model.L1_NAME_H if tag else h
        out.append({"k": "text", "x": item["x"] + 6, "y": row_y, "w": item["w"] - 12, "h": row_h, "t": name, "s": size,
                    "c": rgba(p["name"]), "a": "l"})
        if nv.get("count"):
            out.append({"k": "text", "x": item["x"] + 6, "y": row_y, "w": item["w"] - 12, "h": row_h, "t": nv["count"],
                        "s": size, "c": rgba(p["count"]), "a": "r"})
        if tag and nv.get("tag"):
            out.append({"k": "img", "x": item["x"] + 6, "y": row_y + row_h, "h": tree_model.TAG_ROW_H,
                        "img": "tree_" + nv["tag"], "alpha": p["tag_alpha"]})

    def band_items(bands, prefix=""):
        for b in bands:
            for it in b["items"]:
                key = it["key"]
                if it["kind"] == "line":
                    col = v["lines"].get(prefix + key)
                    if col is not None:
                        out.append({"k": "fill", "x": it["x"], "y": b["y"], "w": it["w"], "h": b["h"], "c": rgba(col)})
                elif it["kind"] == "root":
                    node(it, b["y"], b["h"], "root", v["nodes"]["root"], "ROOT", 20, center=True)
                elif it["kind"] == "l1":
                    node(it, b["y"], b["h"], key, v["nodes"][key], key.upper(), 24, tag=True)
                else:
                    nv = v["nodes"].get("sub:" + key)
                    if nv is not None:
                        node(it, b["y"], b["h"], key, nv, key, 20)

    band_items(lay["bands"])
    if v["group"]:
        band_items(lay["groups"][v["group"]]["bands"], "g:")
    return out


def minimap_draw(tree, area):
    """그리퍼 카메라 창 오른쪽 위 보드 미니맵(isaac_sim_viewport_display._build_minimap 과 같은 도형, 미니맵 좌표)."""
    mv = tree_model.minimap_view(tree, area)
    w, h = tree_model.MINIMAP_W, tree_model.MINIMAP_H
    lw, sl = tree_model.MINIMAP_LINE_PX, tree_model.MINIMAP_SUBLINE_PX
    out = [{"k": "fill", "x": 0, "y": 0, "w": w, "h": h, "c": rgba(tree_model.MINIMAP_BOARD)}]
    if mv["fill"]:
        q, _, sub = mv["fill"].partition("/")
        x, y, cw, ch = tree_model.minimap_cell(q, sub or None)
        out.append({"k": "fill", "x": x, "y": y, "w": cw, "h": ch, "c": rgba(tree_model.MINIMAP_FILL)})
    # 좌표는 isaac_sim_viewport_display._build_minimap 처럼 정수로 반올림한다(선이 번지지 않게, 같은 round)
    for q in mv["split"]:
        x, y, cw, ch = tree_model.minimap_cell(q)
        out.append({"k": "fill", "x": x + round(cw / 2.0 - sl / 2.0), "y": y, "w": sl, "h": ch, "c": rgba(tree_model.MINIMAP_SUBLINE)})
        out.append({"k": "fill", "x": x, "y": y + round(ch / 2.0 - sl / 2.0), "w": cw, "h": sl, "c": rgba(tree_model.MINIMAP_SUBLINE)})
    line = rgba(tree_model.MINIMAP_LINE)
    out += [{"k": "fill", "x": round(w / 2.0 - lw / 2.0), "y": 0, "w": lw, "h": h, "c": line},             # 가운데 십자
            {"k": "fill", "x": 0, "y": round(h / 2.0 - lw / 2.0), "w": w, "h": lw, "c": line},
            {"k": "fill", "x": 0, "y": 0, "w": w, "h": lw, "c": line}, {"k": "fill", "x": 0, "y": h - lw, "w": w, "h": lw, "c": line},
            {"k": "fill", "x": 0, "y": 0, "w": lw, "h": h, "c": line}, {"k": "fill", "x": w - lw, "y": 0, "w": lw, "h": h, "c": line}]
    return {"show": mv["fill"] is not None, "w": w, "h": h, "items": out}


LEGEND_LABEL = {"placed": "final_placed", "place_failed": "final_dropped",   # :225
                "detach_failed": "final_detach_failed"}


def build_view(n_targets, lay):
    """Unity JsonUtility 가 읽는 모양: 키가 바뀌는 사전 없음, 없음(None) 은 빈 문자열."""
    snap, alive = bus_merge.load()
    files = [r for r in bus_merge.ROLES if os.path.exists(bus_merge._path(r))]
    have = any(alive.values()) or any(os.path.getmtime(bus_merge._path(r)) >= bus_merge.EPOCH for r in files)
    hkey = harvest_key(snap)
    area = "home" if hkey == "stopped" else highlight_key(snap["region"]["name"], snap.get("tree"))
    state = snap["sequence"]["state"]
    idx = BAR_STATES.index(state) if state in BAR_STATES else -1
    outcomes = [o for o in (snap["result"].get("outcomes") or []) if isinstance(o, str)]
    rb = result_bar.view(outcomes, n_targets)
    final = ""
    if snap["result"]["finished"]:
        done, total = snap["result"]["succeeded"], snap["targets"]["total"]
        pct = int(round(100.0 * done / total)) if total > 0 else 0
        final = "%d / %d (%d%%)" % (done, total, pct)
    tree = snap.get("tree")
    label_key = tree_model.board_label_key(tree, area)
    return {
        "t": time.time(), "have_data": bool(have),
        "lamps": {r: bool(alive.get(r)) for r in ("vision", "planner", "controller", "scan")},
        "state": state, "state_color": hexrgba(PHASE_COLOR.get(state, 0xE8EDF5)), "bar_index": idx, "bar_len": len(BAR_STATES),
        "harvest": hkey, "final": final,
        "cells": [{"key": k or "", "c": rgba(result_bar.COLOR[k]) if k else []} for k in rb["cells"]],
        "cell_h": result_bar.CELL_H, "cell_gap": result_bar.CELL_GAP, "cell_r": result_bar.CELL_RADIUS,
        "legend": [{"label": LEGEND_LABEL[k], "c": rgba(result_bar.COLOR[k]), "n": int(rb["counts"].get(k, 0))}
                   for k in result_bar.OUTCOMES],
        "legend_size": result_bar.LEGEND_SIZE,
        "area": area, "lit": sorted(HIGHLIGHT_ON.get(area, set())), "label_key": label_key or "",
        "tree_w": lay["width"], "tree_h": lay["height"], "tree": tree_draw(tree, lay),
        "minimap": minimap_draw(tree, area),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.environ.get("E0509_HUD_VIEW", "/tmp/e0509_hud_view.json"))
    ap.add_argument("--targets", type=int, default=8)
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--once", action="store_true", help="한 번 쓰고 끝낸다(시험용)")
    a = ap.parse_args()
    # Isaac 의 hud.install() 처럼 이 세션 시작 시각 — 그 전에 마지막으로 쓰인 파일(이전 런 잔재)은 없는 것으로 본다
    bus_merge.EPOCH = float(os.environ.get("E0509_HUD_EPOCH", time.time()))
    lay = tree_model.tree_layout(TREE_W)
    stop = {"v": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(v=True))
    signal.signal(signal.SIGINT, lambda *_: stop.update(v=True))
    tmp = a.out + ".tmp"
    print(f"[hud_relay] {HUD_DIR} → {a.out} ({a.hz:.0f} Hz), EPOCH {bus_merge.EPOCH:.1f}", flush=True)
    while not stop["v"]:
        try:
            view = build_view(a.targets, lay)
            with open(tmp, "w") as f:
                json.dump(view, f, separators=(",", ":"))
            os.replace(tmp, a.out)
        except Exception as e:      # noqa: BLE001 — 중계가 죽으면 화면이 멈추므로 한 줄 알리고 계속
            print(f"[hud_relay] 오류: {e!r}", flush=True)
        if a.once:
            break
        time.sleep(1.0 / a.hz)
    return 0


if __name__ == "__main__":
    sys.exit(main())
