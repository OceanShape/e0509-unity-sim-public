#!/usr/bin/env python3
"""가짜 `/dsr01/joint_states` 퍼블리셔 — Isaac 없이 M1 미러를 바로 확인하는 용도 (단계 6-1).

Isaac 이 내는 모양을 그대로 흉내 낸다 (CONTRACT §2-2, §3-1):
  - 토픽 `/dsr01/joint_states`, sensor_msgs/JointState, 60 Hz (Isaac 물리 스텝 주기)
  - name 은 Isaac PhysX DOF 순서 10개 고정: joint_1..joint_6, rh_l1, rh_p12_rn, rh_l2, rh_r2
  - position 만 채운다 (rad). Isaac 발행에는 velocity 가 없다 (CONTRACT §2-5-5)
  - QoS 는 기본값 (depth 10, RELIABLE, VOLATILE = CONTRACT §7)

움직임: 시작 자세(overview, CONTRACT §3-6)를 `--hold` 초 유지 → `--ramp` 초 동안 사인파 중심으로
선형 보간 → 관절마다 위상이 다른 사인파. 중심·진폭은 관절별로 한계 안쪽에 둔다.
시작 자세 J2 −94.92° 는 한계(±95°)까지 0.08° 라 그 자리를 중심으로 흔들면 넘는다(CONTRACT §9-1).
그래서 J2 중심은 −65°, J3 중심은 105° 로 한계에서 떨어뜨렸다.

`--limit-test` 를 주면 사인파 구간에서 주기마다 J3 를 한계(135°) 위로 2° 넘기는 구간을 넣는다.
JointLimitMonitor 가 위반을 세는지 확인하는 용도다. 기본은 끔(한계 안쪽만).

쓰는 법:
  source /opt/ros/humble/setup.bash
  python3 -B scripts/fake_joint_states.py [--rate 60] [--period 8] [--hold 2] [--ramp 2] [--limit-test]
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Isaac `/dsr01/joint_states` 의 name 순서 (CONTRACT §2-2). 받는 쪽(Unity)은 이름으로 매핑하지만
# 모양을 Isaac 과 같게 내야 bag 재생과 가짜 퍼블리셔가 같은 코드 경로를 지난다.
NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
         "rh_l1", "rh_p12_rn", "rh_l2", "rh_r2"]

# 하드 한계 (deg). CONTRACT §2-4 Isaac 합성값. 그리퍼는 0–1.1 rad = 0–63.03°.
LIMIT_DEG = {
    "joint_1": (-360.0, 360.0), "joint_2": (-95.0, 95.0), "joint_3": (-135.0, 135.0),
    "joint_4": (-360.0, 360.0), "joint_5": (-135.0, 135.0), "joint_6": (-360.0, 360.0),
    "rh_l1": (0.0, 63.03), "rh_p12_rn": (0.0, 63.03), "rh_l2": (0.0, 63.03), "rh_r2": (0.0, 63.03),
}

# 시작 자세 = overview (CONTRACT §3-6). 그리퍼 네 관절 0.926 rad = 53.05°.
START_DEG = {
    "joint_1": 87.98, "joint_2": -94.92, "joint_3": 129.89,
    "joint_4": 175.94, "joint_5": -31.34, "joint_6": 93.42,
    "rh_l1": 53.05, "rh_p12_rn": 53.05, "rh_l2": 53.05, "rh_r2": 53.05,
}

# 사인파 중심·진폭 (deg). 중심 ± 진폭이 한계 안쪽에 들도록 잡았다 (아래 _check_envelope 가 확인).
SINE_DEG = {                 # (center, amplitude)
    "joint_1": (87.98, 30.0),
    "joint_2": (-65.0, 25.0),    # 시작 −94.92 는 한계 0.08° 앞 → 중심을 안쪽으로
    "joint_3": (105.0, 25.0),    # 시작 129.89 는 한계 5.1° 앞 → 중심을 안쪽으로
    "joint_4": (175.94, 30.0),
    "joint_5": (-31.34, 30.0),
    "joint_6": (93.42, 30.0),
    "rh_l1": (31.5, 23.0), "rh_p12_rn": (31.5, 23.0), "rh_l2": (31.5, 23.0), "rh_r2": (31.5, 23.0),
}
# 그리퍼 네 관절은 한 위상으로 같이 움직인다. 실제 그리퍼·가상 제어기는 네 관절에 같은 값을 내고(Isaac bag: 같은 순간 최대 차 0.035°),
# URDF 순기구학으로 같은 각이면 61.88° 까지 좌우 손가락이 닿지 않는다. 관절마다 위상을 달리 주면(10-21 까지) 근위·원위 관절이
# 최대 13° 어긋나 손가락이 서로 파고든다(10-21 발견, 겹침 정점 73개). 31.5±23° → 8.5–54.5°, 손가락 간격 ≥ 15 mm.
GRIPPER = ("rh_l1", "rh_p12_rn", "rh_l2", "rh_r2")


def _check_envelope():
    for n, (c, a) in SINE_DEG.items():
        lo, hi = LIMIT_DEG[n]
        if not (lo < c - a and c + a < hi):
            raise SystemExit(f"사인파 범위가 한계를 넘는다: {n} [{c-a:.2f}, {c+a:.2f}] vs ({lo}, {hi})")


class FakeJointStates(Node):
    def __init__(self, args):
        super().__init__("fake_joint_states")
        self.args = args
        self.pub = self.create_publisher(JointState, args.topic, 10)
        self.t0 = time.monotonic()
        self.count = 0
        self.last_report = self.t0
        self.timer = self.create_timer(1.0 / args.rate, self.tick)
        self.get_logger().info(
            f"{args.topic} @ {args.rate} Hz, period {args.period}s, hold {args.hold}s, ramp {args.ramp}s, "
            f"limit-test={'on' if args.limit_test else 'off'}")

    def pose_deg(self, t):
        a = self.args
        if t < a.hold:
            return [START_DEG[n] for n in NAMES]
        ts = t - a.hold                        # 사인파 시간
        w = 2.0 * math.pi / a.period
        out = []
        for i, n in enumerate(NAMES):
            c, amp = SINE_DEG[n]
            phase = 2.0 * math.pi * (NAMES.index(GRIPPER[0]) if n in GRIPPER else i) / len(NAMES)
            # 위상 때문에 sin(phase) 가 0 이 아니므로, 램프 끝의 목표는 "ts=ramp 일 때의 사인파 값".
            target = c + amp * math.sin(w * ts + phase)
            if ts < a.ramp:
                k = ts / a.ramp
                v = (1.0 - k) * START_DEG[n] + k * (c + amp * math.sin(w * a.ramp + phase))
            else:
                v = target
            if a.limit_test and n == "joint_3" and ts >= a.ramp:
                # 주기마다 앞 0.5초를 한계 위 +2° 로 고정 → 위반 1구간/주기, 근처 접근도 함께 생긴다.
                if (ts - a.ramp) % a.period < 0.5:
                    v = LIMIT_DEG[n][1] + 2.0
            out.append(v)
        return out

    def tick(self):
        now = time.monotonic()
        t = now - self.t0
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(NAMES)
        msg.position = [math.radians(d) for d in self.pose_deg(t)]
        # velocity·effort 는 비워 둔다 (Isaac 과 같게)
        self.pub.publish(msg)
        self.count += 1
        if now - self.last_report >= 5.0:
            hz = self.count / (now - self.last_report)
            self.get_logger().info(f"t={t:6.1f}s  {hz:5.1f} Hz  J1..J6 deg = "
                                   + " ".join(f"{d:7.2f}" for d in self.pose_deg(t)[:6]))
            self.count = 0
            self.last_report = now


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--topic", default="/dsr01/joint_states")
    p.add_argument("--rate", type=float, default=60.0, help="발행 Hz (기본 60 = Isaac 물리 스텝)")
    p.add_argument("--period", type=float, default=8.0, help="사인파 주기 s")
    p.add_argument("--hold", type=float, default=2.0, help="시작 자세 유지 s")
    p.add_argument("--ramp", type=float, default=2.0, help="시작 자세 → 사인파 보간 s")
    p.add_argument("--limit-test", action="store_true", help="주기마다 J3 를 한계 위 2° 로 0.5초 고정")
    args = p.parse_args(argv)
    _check_envelope()
    rclpy.init()
    node = FakeJointStates(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C 는 KeyboardInterrupt, run_m1.sh 의 SIGTERM 은 rclpy 가 ExternalShutdownException 으로 올린다
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
