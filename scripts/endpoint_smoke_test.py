#!/usr/bin/env python3
"""ROS-TCP-Endpoint 연결 시험 — Unity 없이 Unity 쪽 역할을 TCP 로 흉내 낸다 (단계 2).

확인하는 것
  1. 접속 직후 엔드포인트가 __handshake 를 보내는가 (버전·프로토콜)
  2. ROS -> Unity : __subscribe 한 JointState 토픽이 TCP 로 오고, 역직렬화하면 이름·값이 같은가
  3. Unity -> ROS : __publish 등록 뒤 TCP 로 보낸 String 이 ROS 구독자에게 도착하는가
  4. 커스텀 서비스 타입(dsr_msgs2/MoveJoint) 이름을 엔드포인트가 풀 수 있는가 (__ros_service 등록 뒤 __error 없음)

쓰는 법 (엔드포인트와 같은 ROS_DOMAIN_ID 로):
  source /opt/ros/humble/setup.bash
  ROS_DOMAIN_ID=77 python3 -B scripts/endpoint_smoke_test.py [--host 127.0.0.1] [--port 10000]
종료 코드 0 = 전부 통과.
"""
import argparse
import json
import socket
import statistics
import struct
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import String

ARM = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
       "rh_l1", "rh_p12_rn", "rh_l2", "rh_r2"]          # CONTRACT §2-2 Isaac 순서


def frame(dest: str, payload: bytes) -> bytes:
    d = dest.encode("utf-8")
    return struct.pack("<I", len(d)) + d + struct.pack("<I", len(payload)) + payload


def syscmd(name: str, params: dict) -> bytes:
    # Unity 커넥터는 JSON 끝에 널 문자를 붙이고, 엔드포인트는 마지막 한 글자를 잘라 낸다
    # (server.py handle_syscommand: data.decode()[:-1]). 같은 형식으로 보낸다.
    return frame(name, json.dumps(params).encode("utf-8") + b"\x00")


def recvall(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("endpoint closed the connection")
        buf.extend(chunk)
    return bytes(buf)


class TcpSide:
    def __init__(self, host, port, timeout_s=10.0):
        t0 = time.time()
        while True:
            try:
                self.sock = socket.create_connection((host, port), timeout=2.0)
                break
            except OSError:
                if time.time() - t0 > timeout_s:
                    raise
                time.sleep(0.3)
        self.sock.settimeout(None)
        self.received = []          # (recv_time, dest, payload)
        self.lock = threading.Lock()
        self.alive = True
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        try:
            while self.alive:
                n = struct.unpack("<I", recvall(self.sock, 4))[0]
                dest = recvall(self.sock, n).decode("utf-8").rstrip("\x00")
                m = struct.unpack("<I", recvall(self.sock, 4))[0]
                payload = recvall(self.sock, m) if m else b""
                with self.lock:
                    self.received.append((time.time(), dest, payload))
        except Exception:
            self.alive = False

    def send(self, data: bytes):
        self.sock.sendall(data)

    def by_dest(self, dest):
        with self.lock:
            return [r for r in self.received if r[1] == dest]


def wait_for(pred, timeout_s, step=0.05):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if pred():
            return True
        time.sleep(step)
    return pred()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=10000)
    ap.add_argument("--down-topic", default="/sim_smoke/joint_states")
    ap.add_argument("--up-topic", default="/sim_smoke/from_unity")
    ap.add_argument("--count", type=int, default=100, help="ROS->Unity 로 보낼 JointState 개수 (50 Hz)")
    args = ap.parse_args()

    results = []

    def check(name, ok, detail=""):
        results.append((name, ok))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))

    rclpy.init()
    node = Node("sim_endpoint_smoke")
    pub = node.create_publisher(JointState, args.down_topic, 10)
    got_up = []
    node.create_subscription(String, args.up_topic, lambda m: got_up.append((time.time(), m.data)), 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    print(f"[smoke] {args.host}:{args.port}  down={args.down_topic}  up={args.up_topic}")
    tcp = TcpSide(args.host, args.port)

    # 1. handshake
    ok = wait_for(lambda: tcp.by_dest("__handshake"), 5.0)
    hs = tcp.by_dest("__handshake")
    detail = hs[0][2].decode("utf-8", "replace") if hs else "없음"
    check("handshake 수신", ok, detail)

    # 2·3·4 등록
    tcp.send(syscmd("__subscribe", {"topic": args.down_topic, "message_name": "sensor_msgs/JointState"}))
    tcp.send(syscmd("__publish", {"topic": args.up_topic, "message_name": "std_msgs/String",
                                   "queue_size": 10, "latch": False}))
    tcp.send(syscmd("__ros_service", {"topic": "/sim_smoke/move_joint", "message_name": "dsr_msgs2/MoveJoint"}))

    # 엔드포인트가 ROS 구독자를 만들 때까지 기다린다 (graph 에 구독자가 보이면)
    ok = wait_for(lambda: pub.get_subscription_count() > 0, 5.0)
    check("엔드포인트가 ROS 쪽 구독자 생성", ok, f"subscription_count={pub.get_subscription_count()}")

    # 2. ROS -> Unity
    sent_t = {}
    for i in range(args.count):
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = ARM
        msg.position = [0.001 * i + 0.1 * k for k in range(len(ARM))]
        sent_t[i] = time.time()
        pub.publish(msg)
        time.sleep(0.02)
    wait_for(lambda: len(tcp.by_dest(args.down_topic)) >= args.count, 3.0)
    down = tcp.by_dest(args.down_topic)
    names_ok, vals_ok, lat = True, True, []
    for t_recv, _, payload in down:
        m = deserialize_message(payload, JointState)
        names_ok &= list(m.name) == ARM
        i = round(m.position[0] / 0.001)
        vals_ok &= abs(m.position[9] - (0.001 * i + 0.9)) < 1e-9
        if i in sent_t:
            lat.append((t_recv - sent_t[i]) * 1000.0)
    check("ROS->Unity JointState 도착", len(down) >= args.count * 0.98, f"{len(down)}/{args.count}")
    check("ROS->Unity 이름·값 일치(역직렬화)", names_ok and vals_ok and bool(down))
    if lat:
        lat.sort()
        p95 = lat[min(len(lat) - 1, int(0.95 * len(lat)))]
        print(f"         지연(발행->TCP 수신, 같은 PC) p50 {statistics.median(lat):.2f} ms · p95 {p95:.2f} ms · max {lat[-1]:.2f} ms")

    # 3. Unity -> ROS
    for k in range(5):
        tcp.send(frame(args.up_topic, serialize_message(String(data=f"sim smoke {k}"))))
        time.sleep(0.05)
    ok = wait_for(lambda: len(got_up) >= 5, 3.0)
    check("Unity->ROS String 도착", ok, f"{len(got_up)}/5 {got_up[0][1] if got_up else ''}")

    # 4. 커스텀 서비스 타입 이름 풀기 — 실패하면 엔드포인트가 __error 를 보낸다
    time.sleep(0.5)
    errs = tcp.by_dest("__error")
    check("dsr_msgs2/MoveJoint 타입 해석(__error 없음)", not errs,
          "; ".join(e[2].decode("utf-8", "replace") for e in errs) if errs else "")

    tcp.alive = False
    try:
        tcp.sock.close()
    except OSError:
        pass
    executor.shutdown()          # 스핀 스레드를 먼저 멈춘 뒤 노드를 지운다 (안 그러면 종료 때 abort)
    spin.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()

    failed = [n for n, ok in results if not ok]
    print(f"[smoke] {'전부 통과' if not failed else 'FAIL ' + str(len(failed)) + '건: ' + ', '.join(failed)}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
