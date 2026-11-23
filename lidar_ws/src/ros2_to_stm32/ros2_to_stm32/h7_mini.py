"""Run the single-point MINI route over the H743 USART3 PC Link."""
import argparse
import json
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .h7_pc_link import PcLink
from .trans import VERIFIED_ODOMETRY_TOPIC, fresh_stamp, read_odometry

READY = 4
RUN = 8
STARTING = 1
WALKING = 2
STOPPING = 8
SESSION_LOST = (4, 5, 6, 7)  # PREEMPTED, TIMEOUT, FAULT, DENIED
TRAVEL_DISTANCE = 1.0
POSITION_TOLERANCE = 0.05
HEADING_TOLERANCE = math.radians(8.0)
POSE_TIMEOUT = 0.5
AT_TARGET = "到达目标"


def route_decision(pose, target):
    x, y, _ = pose.position
    remaining_x = target[0] - x
    remaining_y = target[1] - y
    if math.hypot(remaining_x, remaining_y) <= POSITION_TOLERANCE:
        return AT_TARGET
    remaining_forward = remaining_x * math.cos(target[2]) + remaining_y * math.sin(target[2])
    bearing = math.atan2(remaining_y, remaining_x)
    if (remaining_forward <= 0
            or abs(math.remainder(bearing - pose.yaw, math.tau)) >= HEADING_TOLERANCE):
        return '请用遥控器对准目标，H743 PC Link 仅支持直行'
    return None


class H7Mini(Node):
    def __init__(self):
        super().__init__('h7_mini')
        self.device = self.declare_parameter('port', '/dev/ttyUSB0').value
        self.target = None
        self.pose = None
        self.pose_at = float('-inf')
        self.board = None
        self.link = None
        self.retry_at = 0.0
        self.armed = False
        self.moving = False
        self.reached = False
        self.reason = '等待 H743 USART3'
        self.state_pub = self.create_publisher(String, '/motor_state', 10)
        self.create_subscription(Odometry, VERIFIED_ODOMETRY_TOPIC, self.on_pose, 1)
        self.create_service(SetBool, '/automation/enable', self.enable)
        self.create_service(Trigger, '/automation/reset_route', self.reset_route)
        self.create_timer(0.1, self.tick)

    def on_pose(self, message):
        if (message.header.frame_id, message.child_frame_id) != ('odom', 'livox_frame'):
            return
        pose = read_odometry(message)
        if pose is None or (self.pose and pose.stamp_ns <= self.pose.stamp_ns):
            return
        if self.fresh_pose(pose):
            self.pose = pose
            self.pose_at = time.monotonic()

    def fresh_pose(self, pose):
        return pose is not None and fresh_stamp(
            pose.stamp_ns, self.get_clock().now().nanoseconds, POSE_TIMEOUT, 0.1)

    def pose_problem(self):
        if time.monotonic() - self.pose_at > POSE_TIMEOUT or not self.fresh_pose(self.pose):
            return '定位超时'
        if self.target is not None:
            return route_decision(self.pose, self.target)
        return None

    def enable(self, request, response):
        if not request.data:
            self.stop('人工暂停')
            response.success = self.link is not None
            response.message = self.reason
            return response
        if self.armed:
            response.success = True
            response.message = 'H743 MINI 已启用'
            return response

        reason = '路线已完成，请先重置' if self.reached else self.pose_problem()
        if self.board is None or self.board.state not in (READY, RUN) or self.board.fault:
            reason = '等待 H743 进入 READY 或 RUN，且无故障'
        elif self.board.phase in (STARTING, WALKING, STOPPING):
            reason = '等待板端动作或收步结束'
        response.success = reason is None
        response.message = reason or 'H743 MINI 已启用'
        if response.success:
            if self.target is None:
                x, y, _ = self.pose.position
                yaw = self.pose.yaw
                self.target = (x + TRAVEL_DISTANCE * math.cos(yaw),
                               y + TRAVEL_DISTANCE * math.sin(yaw), yaw)
            self.armed = True
            self.reason = response.message
        return response

    def reset_route(self, _request, response):
        response.success = not self.armed and not self.moving
        if response.success:
            self.reached = False
            self.target = None
            self.reason = '路线已重置，等待 arm'
        response.message = self.reason if response.success else '请先暂停路线'
        return response

    def disconnect(self, reason):
        if self.link is not None:
            self.link.close()
        self.link = self.board = None
        self.armed = self.moving = False
        self.retry_at = time.monotonic() + 1.0
        self.reason = reason

    def stop(self, reason):
        self.armed = False
        self.reason = reason
        if self.moving:
            try:
                self.link.stand()
            except (OSError, ValueError) as error:
                self.disconnect(f'{reason}，收步未获确认：{error}')
        self.moving = False

    def run_cycle(self):
        if self.link is None:
            if time.monotonic() < self.retry_at:
                return
            self.link = PcLink(self.device)
            self.link.connect()
            if self.reason == '等待 H743 USART3':
                self.reason = '已连接 H743，等待 arm'

        if self.armed:
            avoid_motion = self.pose_problem()
            if avoid_motion:
                self.reached = avoid_motion == AT_TARGET
                self.stop(avoid_motion)
                if self.link is None:
                    return

        self.board = self.link.status()
        if self.board.phase in SESSION_LOST or self.board.fault:
            self.disconnect(f'H743 会话已结束，phase={self.board.phase}, fault={self.board.fault}')
            return
        if self.armed and not self.moving:
            if self.board.state not in (READY, RUN):
                self.stop('H743 尚未就绪')
                return
            self.moving = True  # Release our session even if the TROT ACK is lost.
            self.link.start_trot()
            self.reason = '已请求 TROT 前进'

    def tick(self):
        try:
            self.run_cycle()
        except (OSError, ValueError) as error:
            self.stop(f'H743 通讯或指令异常：{error}')
            if self.link is not None or time.monotonic() >= self.retry_at:
                self.disconnect(self.reason)
        board = self.board
        state = dict(armed=self.armed, reason=self.reason, connected=board is not None,
                     busy=self.moving or bool(board and board.phase == STOPPING),
                     gait=board.gait if board else None,
                     phase=board.phase if board else None,
                     state=board.state if board else None,
                     fault=board.fault if board else None)
        self.state_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))

    def destroy_node(self):
        self.stop('H743 MINI 已退出')
        if self.link is not None:
            self.link.close()
        return super().destroy_node()


def status_only(args):
    parser = argparse.ArgumentParser(description='读取 H743 USART3 PC Link 状态')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    options = parser.parse_args(args)
    try:
        with PcLink(options.port) as link:
            link.connect()
            print(json.dumps(link.status()._asdict()))
    except (OSError, ValueError) as error:
        parser.exit(1, f'{error}\n')


def main():
    if sys.argv[1:2] == ['--status']:
        status_only(sys.argv[2:])
        return
    rclpy.init()
    node = H7Mini()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
