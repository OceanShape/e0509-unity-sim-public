#!/usr/bin/env python3
"""플랜트가 overview 자세로 /dsr01/joint_states 를 내는지 확인한다 (M2 시작 게이트, CONTRACT §3-6·§5-3).

스캔 실행기의 시작 게이트와 같은 기준: 팔 6축이 overview 와 ±1.0° 안, J1·J4·J6 는 360° 등가 허용(SE:550,565-569).
  python3 -B scripts/check_overview.py [--timeout 60] [--tol 1.0]
종료 코드 0 = 통과, 1 = 자세 불일치, 2 = 시간 안에 메시지 없음.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

OVERVIEW = {"joint_1": 87.98, "joint_2": -94.92, "joint_3": 129.89, "joint_4": 175.94, "joint_5": -31.34, "joint_6": 93.42}
WRAP_OK = {"joint_1", "joint_4", "joint_6"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/dsr01/joint_states")
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--tol", type=float, default=1.0)
    a = ap.parse_args()
    rclpy.init()
    node = Node("check_overview")
    got = {}

    def cb(msg):
        if not got:
            got.update({n: math.degrees(p) for n, p in zip(msg.name, msg.position)})
            got["_names"] = list(msg.name)

    node.create_subscription(JointState, a.topic, cb, 10)
    t0 = time.time()
    while not got and time.time() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if not got:
        print(f"[check_overview] {a.timeout:.0f}s 안에 {a.topic} 메시지 없음")
        return 2
    print(f"[check_overview] name 순서: {got['_names']}")
    bad = []
    for j, ref in OVERVIEW.items():
        v = got.get(j)
        if v is None:
            bad.append(f"{j} 없음")
            continue
        d = v - ref
        if j in WRAP_OK:
            d = (d + 180.0) % 360.0 - 180.0
        ok = abs(d) <= a.tol
        print(f"  {j}: {v:9.3f}°  overview {ref:8.2f}°  차 {d:+.3f}°  {'OK' if ok else '!!'}")
        if not ok:
            bad.append(j)
    if bad:
        print(f"[check_overview] 불일치: {bad}")
        return 1
    print(f"[check_overview] overview ±{a.tol}° 통과")
    return 0


if __name__ == "__main__":
    sys.exit(main())
