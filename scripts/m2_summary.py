#!/usr/bin/env python3
"""M2 런 요약 (CONTRACT §6 M2 완료 판정). 런 폴더의 노드 로그·플랜트 보고서·ROS 주기를 모아 summary.json/.txt 를 쓴다.

판정 근거 (서비스 응답은 도착과 무관하게 성공이라 쓰지 않는다):
  - 가상 제어기 bridge.log: ARM_ARRIVAL / ARM_ARRIVAL_STALLED / ARM_ARRIVAL_TIMEOUT 따로 센다. 목표 TIMEOUT 0.
  - 스캔 scan.log: START_ACCEPTED, AT_SCAN_POSE, PICK_TRIGGER→PICK_ATTEMPT_DONE 시간, SCAN_COMPLETE, EXEC_TIMEOUT, START_REJECTED
    픽이 한 번도 없는 런은 완주로 보지 않는다(overview 스캔에서 익은 과실 0개면 스캔 실행기가 3 s 만에 SCAN_COMPLETE 를 낸다).
    시작한 픽은 전부 PICK_ATTEMPT_DONE 으로 끝나야 한다. overview 스캔의 분면별 익은 과실 수(OVERVIEW_SCAN)도 적는다.
  - 파지 판정은 가상 제어기의 GRASP_ATTACH(접촉)/GRASP_EMPTY(빈손) 와 플래너의 GRASP_CONTACT_DETECTED/GRASP_EMPTY 로만 적는다.
    "수확 성공"이라 쓰지 않는다(Unity 는 파지를 판정하지 않는다).
비교용으로 딸기 리포의 Isaac 완주 런 하나를 같은 방식으로 센다(읽기만, 기본 run_logs/20261014_002919, 스캔 속도 120/180).
  python3 -B scripts/m2_summary.py logs/m2/<run> [--ref ~/strawberry_grasp_environment/run_logs/20261014_002919]
"""
import argparse
import json
import os
import re
import sys

TS = re.compile(r"\[(\d{10}\.\d+)\]")


def lines(path):
    try:
        with open(path, errors="ignore") as f:
            return f.readlines()
    except FileNotFoundError:
        return []


def count(ls, pat):
    r = re.compile(pat)
    return sum(1 for l in ls if r.search(l))


def first_ts(ls, pat):
    r = re.compile(pat)
    for l in ls:
        if r.search(l):
            m = TS.search(l)
            if m:
                return float(m.group(1))
    return None


def all_ts(ls, pat):
    r = re.compile(pat)
    out = []
    for l in ls:
        if r.search(l):
            m = TS.search(l)
            if m:
                out.append(float(m.group(1)))
    return out


def analyze(run):
    br, sc, pl = (lines(os.path.join(run, n)) for n in ("bridge.log", "scan.log", "planner.log"))
    pick_t, pick_d = all_ts(sc, r"PICK_TRIGGER"), all_ts(sc, r"PICK_ATTEMPT_DONE")
    st, en = first_ts(sc, r"START_ACCEPTED"), first_ts(sc, r"\bSCAN_COMPLETE\b")
    picks = [round(d - t, 1) for t, d in zip(pick_t, pick_d)]
    ov = next((re.findall(r"\b(nw|ne|se|sw):(\d+)", l) for l in sc if re.search(r"\bOVERVIEW_SCAN\b", l)), None)
    return {
        "bridge": {
            "ARM_ARRIVAL": count(br, r"\bARM_ARRIVAL\b"),
            "ARM_ARRIVAL_STALLED": count(br, r"\bARM_ARRIVAL_STALLED\b"),
            "ARM_ARRIVAL_TIMEOUT": count(br, r"\bARM_ARRIVAL_TIMEOUT\b"),
            "GRASP_JUDGE": count(br, r"\bGRASP_JUDGE\b"),
            "GRASP_ATTACH": count(br, r"\bGRASP_ATTACH\b"),
            "GRASP_EMPTY": count(br, r"\bGRASP_EMPTY\b"),
            "GRASP_RELEASE": count(br, r"\bGRASP_RELEASE\b"),
            "MOVEJ_OVER_DOOSAN_MOVEIT": count(br, r"\bMOVEJ_OVER_DOOSAN_MOVEIT\b"),
        },
        "scan": {
            "START_ACCEPTED": count(sc, r"\bSTART_ACCEPTED\b"),
            "START_REJECTED": count(sc, r"\bSTART_REJECTED\b"),
            "AT_SCAN_POSE": count(sc, r"\bAT_SCAN_POSE\b"),
            "EXEC_TIMEOUT": count(sc, r"\bEXEC_TIMEOUT\b"),
            "PICK_TRIGGER": len(pick_t),
            "PICK_ATTEMPT_DONE": len(pick_d),
            "SCAN_COMPLETE": count(sc, r"\bSCAN_COMPLETE\b"),
            "READY_FOR_NEXT_START": count(sc, r"\bREADY_FOR_NEXT_START\b"),
            "start_to_complete_s": round(en - st, 1) if st and en else None,
            "pick_durations_s": picks,
            "pick_max_s": max(picks) if picks else None,
            "overview_ripe": {k: int(v) for k, v in ov} if ov else None,
        },
        "planner": {
            "GRASP_CONTACT_DETECTED": count(pl, r"\bGRASP_CONTACT_DETECTED\b"),
            "GRASP_EMPTY": count(pl, r"\bGRASP_EMPTY\b"),
            "PLACE_GATE_BLOCKED": count(pl, r"\bPLACE_GATE_BLOCKED\b"),
        },
    }


def load_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--ref", default=os.path.expanduser("~/strawberry_grasp_environment/run_logs/20261014_002919"))
    a = ap.parse_args()
    run = a.run
    s = analyze(run)
    s["ros_rate"] = load_json(os.path.join(run, "ros_rate.json"))
    s["unity_plant"] = load_json(os.path.join(run, "unity_plant_report.json"))
    new_path = os.path.join(run, "strawberry_status_new.txt")
    s["strawberry_repo_unchanged"] = not (os.path.exists(new_path) and os.path.getsize(new_path) > 0) \
        if os.path.exists(os.path.join(run, "strawberry_status_after.txt")) else None
    s["reference_isaac_run"] = {"path": a.ref, **analyze(a.ref)} if os.path.isdir(a.ref) else None

    b, c = s["bridge"], s["scan"]
    js = (s["ros_rate"] or {}).get("joint_states", {})
    verdict = {
        "arrival_timeout_zero": b["ARM_ARRIVAL_TIMEOUT"] == 0 and (b["ARM_ARRIVAL"] + b["ARM_ARRIVAL_STALLED"]) > 0,
        "sequence_complete": c["SCAN_COMPLETE"] >= 1 and c["EXEC_TIMEOUT"] == 0 and c["START_REJECTED"] == 0,
        "picks_attempted": c["PICK_TRIGGER"] >= 1 and c["PICK_ATTEMPT_DONE"] == c["PICK_TRIGGER"],
        "state_rate_ok": bool(js) and js.get("gaps_over_0.3s", 1) == 0 and js.get("mean_hz", 0) >= 50,
    }
    verdict["M2_complete"] = all(verdict.values())
    s["verdict"] = verdict
    with open(os.path.join(run, "summary.json"), "w") as f:
        json.dump(s, f, indent=1, ensure_ascii=False)

    up = s["unity_plant"] or {}
    ref = s["reference_isaac_run"]
    ovr = c["overview_ripe"]
    txt = [
        f"M2 런 요약 — {run}",
        f"판정: M2 완료 {'예' if verdict['M2_complete'] else '아니오'}  "
        f"(도착 타임아웃 0: {verdict['arrival_timeout_zero']}, 시퀀스 완주: {verdict['sequence_complete']}, "
        f"픽 시도·종료: {verdict['picks_attempted']}, 상태 주기: {verdict['state_rate_ok']})",
        f"가상 제어기 도착: ARM_ARRIVAL {b['ARM_ARRIVAL']} · STALLED {b['ARM_ARRIVAL_STALLED']} · TIMEOUT {b['ARM_ARRIVAL_TIMEOUT']}",
        f"파지 판정(가상 제어기): 접촉 GRASP_ATTACH {b['GRASP_ATTACH']} · 빈손 GRASP_EMPTY {b['GRASP_EMPTY']} · RELEASE {b['GRASP_RELEASE']}",
        f"스캔: START_ACCEPTED {c['START_ACCEPTED']} · AT_SCAN_POSE {c['AT_SCAN_POSE']} · PICK_TRIGGER {c['PICK_TRIGGER']} · SCAN_COMPLETE {c['SCAN_COMPLETE']} · "
        f"EXEC_TIMEOUT {c['EXEC_TIMEOUT']} · 시작→완료 {c['start_to_complete_s']} s",
        f"overview 스캔 익은 과실: {sum(ovr.values()) if ovr else None} ({' · '.join(f'{k} {v}' for k, v in ovr.items()) if ovr else '로그 없음'})",
        f"픽 시간(PICK_TRIGGER→DONE) s: {c['pick_durations_s']} (최대 {c['pick_max_s']}, 스캔 대기 상한 60 s)",
        f"/dsr01/joint_states (ROS 쪽): {js.get('mean_hz')} Hz, 최대 간격 {js.get('max_gap_s')} s, 0.3 s 넘는 간격 {js.get('gaps_over_0.3s')}",
        f"Unity 플랜트: 명령 {up.get('commands_received')} · 상태 발행 {up.get('state_published')} · 한계 위반 {up.get('violations')} · 근처 {up.get('near_limit_episodes')} · 판정 값 {up.get('monitor_value_source')}",
        f"딸기 리포 변화 없음: {s['strawberry_repo_unchanged']}",
    ]
    if ovr is not None and sum(ovr.values()) == 0:
        txt.append("!! overview 스캔에서 익은 과실 0개 — 에디터 플랜트를 같은 Play 로 이어 돌리면 앞 런에서 붙거나 수확된 과실이 "
                   "/isaac_sim/strawberries 에서 빠진 채로 남는다. Play 를 다시 시작한 뒤 돌린다.")
    if ref:
        rb, rc = ref["bridge"], ref["scan"]
        txt.append(f"참고 Isaac 런 {os.path.basename(ref['path'])} (스캔 120/180): ARM_ARRIVAL {rb['ARM_ARRIVAL']} · STALLED {rb['ARM_ARRIVAL_STALLED']} · "
                   f"TIMEOUT {rb['ARM_ARRIVAL_TIMEOUT']} · 접촉 {rb['GRASP_ATTACH']} · 빈손 {rb['GRASP_EMPTY']} · PICK {rc['PICK_TRIGGER']} · "
                   f"시작→완료 {rc['start_to_complete_s']} s · 픽 {rc['pick_durations_s']}")
    with open(os.path.join(run, "summary.txt"), "w") as f:
        f.write("\n".join(txt) + "\n")
    print("\n".join(txt))
    return 0 if verdict["M2_complete"] else 1


if __name__ == "__main__":
    sys.exit(main())
