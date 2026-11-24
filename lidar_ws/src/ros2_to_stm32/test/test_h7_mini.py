import json
import math
import os
import pty
import select
import threading
import time
import tty

import pytest
import rclpy
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, Trigger

from ros2_to_stm32.h7_mini import H7Mini


def wire(body):
    # Independent CRC-16/CCITT implementation for the simulated board.
    crc = 0xFFFF
    for byte in body:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return body + f'*{crc:04X}\n'.encode('ascii')


class VirtualH743:
    session = b'12345678'

    def __init__(self, fd):
        self.fd = fd
        self.received = []
        self.phase = 0
        self.last_sequence = 0
        self.drop_trot_ack = False
        self.closed = threading.Event()
        self.thread = threading.Thread(target=self.serve, daemon=True)

    def serve(self):
        pending = bytearray()
        while not self.closed.is_set():
            if not select.select([self.fd], [], [], 0.02)[0]:
                continue
            pending.extend(os.read(self.fd, 256))
            while (end := pending.find(b'\n')) >= 0:
                frame = bytes(pending[:end + 1])
                del pending[:end + 1]
                body = frame[:-6]
                assert frame == wire(body)
                operation = body[:2]
                self.received.append(operation)
                if operation == b'H1':
                    self.last_sequence = 0
                    reply = b'Y1' + body[2:] + self.session
                else:
                    assert body[2:10] == self.session
                    sequence = int(body[10:14], 16)
                    assert sequence == self.last_sequence + 1
                    self.last_sequence = sequence
                    if operation == b'K1':
                        reply = b'U1' + body[2:] + f'0800{self.phase:02X}00FF00'.encode()
                    elif operation == b'T1':
                        self.phase = 2
                        if self.drop_trot_ack:
                            continue
                        reply = b'A1' + body[2:] + b'01'
                    else:
                        assert operation == b'Z1'
                        self.phase = 3
                        reply = b'A1' + body[2:] + b'00'
                os.write(self.fd, wire(reply))


@pytest.fixture
def h7():
    master, slave = pty.openpty()
    tty.setraw(slave)
    board = VirtualH743(master)
    board.thread.start()
    rclpy.init()
    node = H7Mini()
    node.device = os.ttyname(slave)
    try:
        yield node, board
    finally:
        node.destroy_node()
        rclpy.shutdown()
        board.closed.set()
        board.thread.join(timeout=0.5)
        os.close(master)
        os.close(slave)


def enable(node):
    return node.enable(SetBool.Request(data=True), SetBool.Response()).success


def pose(node, x, yaw=0.0, *, y=0.0, age=0.0):
    message = Odometry()
    message.header.frame_id = 'odom'
    message.child_frame_id = 'livox_frame'
    ns = node.get_clock().now().nanoseconds - round(age * 1e9)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, 10**9)
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(math.radians(yaw) / 2.0)
    message.pose.pose.orientation.w = math.cos(math.radians(yaw) / 2.0)
    node.on_pose(message)


def test_h7_uses_current_position_and_heading_without_ab(h7):
    node, board = h7
    node.tick()
    assert board.received == [b'H1', b'K1']
    assert not enable(node)
    pose(node, 2.0, age=0.6)
    assert not enable(node)
    assert b'T1' not in board.received

    pose(node, 2.0, y=-3.0)
    pose(node, 4.0, 90.0, y=10.0)
    assert enable(node)
    assert node.target == pytest.approx((4.0, 11.0, math.pi / 2))
    node.tick()
    assert board.received.count(b'T1') == 1
    pose(node, 4.0, 90.0, y=10.79)
    node.tick()
    assert b'Z1' not in board.received

    pose(node, 4.0, 90.0, y=11.0)
    node.tick()
    assert board.received.count(b'Z1') == 1
    assert node.reached and not node.armed and node.board.phase == 3
    node.tick()
    assert board.received.count(b'T1') == 1


def test_h7_reaches_target_with_small_lateral_offset(h7):
    node, board = h7
    states = []
    node.state_pub.publish = lambda msg: states.append(json.loads(msg.data))
    node.tick()
    pose(node, 0.0)
    assert enable(node)
    node.tick()

    pose(node, 0.8, y=0.03)
    node.tick()
    assert b'Z1' not in board.received
    assert states[-1]['route'] == pytest.approx(dict(
        forward_m=0.8, remaining_m=0.2, cross_track_m=0.03, heading_error_deg=0.0))

    pose(node, 1.0, y=0.03)
    node.tick()
    assert board.received.count(b'Z1') == 1
    assert node.reached and states[-1]['reason'] == '到达目标'


def test_h7_keeps_moving_at_reported_lateral_offset(h7):
    node, board = h7
    node.tick()
    pose(node, 0.0)
    assert enable(node)
    node.tick()

    pose(node, 0.582, -5.0, y=-0.051)
    node.tick()
    assert node.armed and b'Z1' not in board.received

    pose(node, 0.96, -5.0, y=-0.088)
    node.tick()
    assert node.reached and not node.armed
    assert board.received.count(b'Z1') == 1


@pytest.mark.parametrize(('x', 'y', 'reason'), [
    (0.6, 0.11, '偏离起始直线超过 10 cm'),
    (1.08, 0.0, '已越过目标线'),
])
def test_h7_stops_outside_the_straight_route(h7, x, y, reason):
    node, board = h7
    node.tick()
    pose(node, 0.0)
    assert enable(node)
    node.tick()

    pose(node, x, y=y)
    node.tick()
    assert board.received.count(b'Z1') == 1
    assert not node.armed and not node.reached
    assert reason in node.reason


def test_h7_resume_preserves_target_until_route_reset(h7):
    node, board = h7
    node.tick()
    pose(node, 4.0)
    assert enable(node)
    node.tick()
    pose(node, 4.4)
    node.enable(SetBool.Request(data=False), SetBool.Response())
    assert board.received.count(b'Z1') == 1
    node.tick()
    pose(node, 4.4, 25.0)
    assert not enable(node)
    pose(node, 4.4)
    assert enable(node)
    assert node.target == pytest.approx((5.0, 0.0, 0.0))
    node.tick()
    pose(node, 5.0)
    node.tick()
    assert board.received.count(b'Z1') == 2
    assert node.reached

    assert node.reset_route(Trigger.Request(), Trigger.Response()).success
    pose(node, 5.0, 180.0)
    assert enable(node)
    assert node.target == pytest.approx((4.0, 0.0, math.pi))


@pytest.mark.parametrize('failure', ['stale_pose', 'heading_changed', 'remote_preempted'])
def test_h7_ends_motion_without_replaying_trot(h7, failure):
    node, board = h7
    node.tick()
    pose(node, 0.0)
    assert enable(node)
    node.tick()
    if failure == 'stale_pose':
        node.pose_at = time.monotonic() - 1.0
    elif failure == 'heading_changed':
        pose(node, 0.2, 25.0)
    else:
        board.phase = 4  # PREEMPTED by the physical remote.
    node.tick()
    assert not node.armed
    if failure == 'remote_preempted':
        assert b'Z1' not in board.received
    else:
        assert board.received.count(b'Z1') == 1
    pose(node, 0.3)
    node.tick()
    assert board.received.count(b'T1') == 1


def test_h7_requests_stand_if_trot_ack_is_lost(h7):
    node, board = h7
    node.tick()
    pose(node, 0.0)
    assert enable(node)
    board.drop_trot_ack = True
    node.tick()
    assert not node.armed and node.link is None
    assert board.received[-2:] == [b'T1', b'Z1']
