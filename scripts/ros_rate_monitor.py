#!/usr/bin/env python3
"""ROS 쪽에서 플랜트 토픽 주기를 잰다 (M2 근거, CONTRACT §3-6 '발행 주기 하한').

/dsr01/joint_states: 수신 수, 평균 Hz, 최대 간격, 0.3 s 넘는 간격 수(가상 제어기 정체 판정이 관대해지는 기준).
/isaac_sim/strawberries: 수신 수, 평균 주기, 과실 수.
SIGTERM/SIGINT 를 받으면 JSON 을 쓰고 끝낸다.
  python3 -B scripts/ros_rate_monitor.py --out logs/m2/<run>/ros_rate.json
"""
import argparse
import json
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from sensor_msgs.msg import JointState


class RateMon(Node):
    def __init__(self):
        super().__init__("ros_rate_monitor")
        self.js_times = []
        self.js_names = None
        self.big_gaps = []
        self.last_stamp = 0.0
        self.berry_times = []
        self.berry_counts = set()
        self.create_subscription(JointState, "/dsr01/joint_states", self.on_js, 50)
        self.create_subscription(PoseArray, "/isaac_sim/strawberries", self.on_berry, 10)

    def on_js(self, msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.js_times:
            gap = now - self.js_times[-1]
            if gap > 0.1:
                # 수신 간격과 Unity 가 찍은 발행 시각 간격을 같이 남긴다: 둘 다 크면 Unity 쪽 끊김, 수신만 크면 전달 구간
                self.big_gaps.append({"wall": round(time.time(), 3), "recv_gap_s": round(gap, 4),
                                      "stamp_gap_s": round(stamp - self.last_stamp, 4)})
        self.js_times.append(now)
        self.last_stamp = stamp
        if self.js_names is None:
            self.js_names = list(msg.name)

    def on_berry(self, msg):
        self.berry_times.append(time.monotonic())
        self.berry_counts.add(len(msg.poses))


def stats(ts, gap_thresh=None):
    if len(ts) < 2:
        return {"count": len(ts)}
    gaps = [b - a for a, b in zip(ts, ts[1:])]
    dur = ts[-1] - ts[0]
    s = {"count": len(ts), "duration_s": round(dur, 2), "mean_hz": round((len(ts) - 1) / dur, 3),
         "max_gap_s": round(max(gaps), 4), "p99_gap_s": round(sorted(gaps)[int(0.99 * (len(gaps) - 1))], 4)}
    if gap_thresh is not None:
        s[f"gaps_over_{gap_thresh}s"] = sum(1 for g in gaps if g > gap_thresh)
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    rclpy.init()
    node = RateMon()
    stop = {"v": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(v=True))
    signal.signal(signal.SIGINT, lambda *_: stop.update(v=True))
    while not stop["v"]:
        rclpy.spin_once(node, timeout_sec=0.1)
    res = {"joint_states": stats(node.js_times, 0.3), "joint_state_names": node.js_names,
           "strawberries": stats(node.berry_times), "berry_counts_seen": sorted(node.berry_counts),
           "joint_state_gaps_over_0.1s": node.big_gaps}
    with open(a.out, "w") as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    print(json.dumps(res, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
