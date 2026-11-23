import math
import time
from typing import NamedTuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray

from .zone_manager import ZoneManager, zone_config_path


VERIFIED_ODOMETRY_TOPIC = '/Odometry/verified'


class AcceptedOdometry(NamedTuple):
    stamp_ns: int
    position: tuple[float, float, float]
    yaw: float


def read_odometry(message):
    pose = message.pose.pose
    position = (pose.position.x, pose.position.y, pose.position.z)
    rotation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    if not all(math.isfinite(value) for value in (*position, *rotation)):
        return None
    norm = math.hypot(*rotation)
    if not math.isfinite(norm) or norm < 1e-6:
        return None
    x, y, z, w = (value / norm for value in rotation)
    yaw = math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
    stamp = message.header.stamp
    return AcceptedOdometry(stamp.sec * 1_000_000_000 + stamp.nanosec, position, yaw)


def fresh_stamp(stamp_ns, now_ns, max_age, future_tolerance):
    age = (now_ns - stamp_ns) / 1_000_000_000
    return stamp_ns > 0 and -future_tolerance <= age <= max_age


class TopicRenamer(Node):
    def __init__(self):
        super().__init__('topic_trans_node')
        self.x1 = 0.0
        self.y1 = 0.0
        self.x2 = 0.0
        self.y2 = 0.0
        self.x_p = 0.0
        self.y_p = 0.0
        self.z_p = 0.0
        self.x_trans = 0.0
        self.y_trans = 0.0
        self.first_captured = False
        self.second_captured = False
        self.last_odometry_time = float('-inf')
        self._last_odometry = None
        self._recovering = False
        self._recovery_sample = None
        self.odom_max_age = self._odom_parameter('odom_max_age', 0.5)
        self.odom_future_tolerance = self._odom_parameter('odom_future_tolerance', 0.1)
        self.odom_reset_threshold = self._odom_parameter('odom_reset_threshold', 0.1)
        self.odom_max_gap = self._odom_parameter('odom_max_gap', 1.0)
        self.odom_jump_distance = self._odom_parameter('odom_jump_distance', 0.5)
        self.odom_jump_speed = self._odom_parameter('odom_jump_speed', 3.0)
        self.odom_jump_yaw = math.radians(self._odom_parameter('odom_jump_yaw_deg', 45.0))
        self.odom_jump_yaw_rate = math.radians(
            self._odom_parameter('odom_jump_yaw_rate_deg_s', 180.0))
        self.calibration_publisher = self.create_publisher(Bool, '/calibration_ready', 1)

        self.zm = ZoneManager(zone_config_path())

        self.task_param_subscription = self.create_subscription(
            Float32MultiArray,
            '/task_params',
            self.task_param_callback,
            10)
        self.subscription = self.create_subscription(
            Odometry,  
            '/Odometry',
            self.Odometry_callback,
            10  
        )

        self.verified_pose_publisher = self.create_publisher(Odometry, VERIFIED_ODOMETRY_TOPIC, 1)
        self.publisher = self.create_publisher(Float32MultiArray, '/pos_trans', 20)
    def update_zone_data(self, zone_name, new_x, new_y):
        """
        如果你需要在节点运行中修改数据并保存
        """
        # 重新加载最新数据（防止覆盖别人刚写的）
        current_data = self.zm.load_data()
        
        # 修改数据
        current_data["zones"][zone_name]["begin_x"] = new_x
        current_data["zones"][zone_name]["begin_y"] = new_y
        
        # 保存回文件
        self.zm.save_data(current_data)
        self.get_logger().info(f"{zone_name} 数据已更新并保存至文件。")
    def task_param_callback(self, msg):
        if not msg.data:
            return
        if msg.data[0] in (1.0, 2.0) and (
            self._last_odometry is None
            or time.monotonic() - self.last_odometry_time > self.odom_max_age
            or not self._fresh_stamp(self._last_odometry.stamp_ns)
        ):
            self.get_logger().error('没有新鲜里程计，拒绝踩点')
            return
        if msg.data[0] == 1.0:
            self.x1 = self.x_p
            self.y1 = self.y_p
            self.first_captured = True
            self.second_captured = False
            self.calibration_publisher.publish(Bool(data=False))
        elif msg.data[0] == 2.0:
            if not self.first_captured:
                self.get_logger().error('请先采 A 点')
                return
            self.x2 = self.x_p
            self.y2 = self.y_p
            self.second_captured = math.hypot(self.x2 - self.x1, self.y2 - self.y1) >= 0.05
            if not self.second_captured:
                self.get_logger().error('A/B 点间距不足 5cm，请重新采 B 点')
        elif msg.data[0] == 3.0:
            self.update_zone_data("木桥A",self.x_trans,self.y_trans)
        elif msg.data[0] == 4.0:
            self.update_zone_data("木桥B",self.x_trans,self.y_trans)
        elif msg.data[0] == 5.0:
            self.update_zone_data("大斜坡",self.x_trans,self.y_trans)
        elif msg.data[0] == 6.0:
            self.update_zone_data("限高栏",self.x_trans,self.y_trans)
        elif msg.data[0] == 7.0:
            self.update_zone_data("沙坑",self.x_trans,self.y_trans)
        elif msg.data[0] == 8.0:
            self.update_zone_data("绕杆",self.x_trans,self.y_trans)
        elif msg.data[0] == 9.0:
            self.update_zone_data("高墙",self.x_trans,self.y_trans)
        elif msg.data[0] == 10.0:
            self.update_zone_data("台阶",self.x_trans,self.y_trans)

    def _odom_parameter(self, name, default):
        value = self.declare_parameter(name, default).value
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def _expected_odom_frames(self):
        return 'odom', 'livox_frame'

    def _fresh_stamp(self, stamp_ns):
        return fresh_stamp(stamp_ns, self.get_clock().now().nanoseconds,
                           self.odom_max_age, self.odom_future_tolerance)

    @staticmethod
    def _pose_change(previous, current):
        distance = math.dist(previous.position, current.position)
        turn = abs(math.remainder(current.yaw - previous.yaw, math.tau))
        return distance, turn

    def _exceeds_jump_limit(self, distance, turn, elapsed):
        return (distance > max(self.odom_jump_distance, self.odom_jump_speed * elapsed)
                or turn > max(self.odom_jump_yaw, self.odom_jump_yaw_rate * elapsed))

    def _begin_recovery(self, reason):
        # Keep the last trusted pose in the estimator frame as the recovery anchor.
        self._recovering = True
        self._recovery_sample = None
        self.last_odometry_time = float('-inf')
        self.calibration_publisher.publish(Bool(data=False))
        self.get_logger().warning(f'{reason}，等待位姿恢复连续')

    def _recovery_confirmed(self, current, distance, turn):
        # Do not grow the recovery envelope with the length of the outage.
        if distance > self.odom_jump_distance or turn > self.odom_jump_yaw:
            self._recovery_sample = None
            return False

        candidate = self._recovery_sample
        if candidate is not None and current.stamp_ns <= candidate.stamp_ns:
            return False
        self._recovery_sample = current
        if candidate is None or not self._fresh_stamp(candidate.stamp_ns):
            return False

        elapsed = (current.stamp_ns - candidate.stamp_ns) / 1_000_000_000
        distance, turn = self._pose_change(candidate, current)
        if self._exceeds_jump_limit(distance, turn, elapsed):
            return False
        self._recovering = False
        self._recovery_sample = None
        self.get_logger().info('里程计连续性已恢复')
        return True

    def Odometry_callback(self, pos_msg):
        if (pos_msg.header.frame_id, pos_msg.child_frame_id) != self._expected_odom_frames():
            self.get_logger().warning('丢弃坐标系不符的里程计', throttle_duration_sec=5.0)
            return

        current = read_odometry(pos_msg)
        if current is None:
            return
        if not self._fresh_stamp(current.stamp_ns):
            self.get_logger().warning('丢弃缺少有效时间戳、过期或超前的里程计', throttle_duration_sec=5.0)
            return

        previous = self._last_odometry
        if previous is not None:
            elapsed = (current.stamp_ns - previous.stamp_ns) / 1_000_000_000
            if elapsed <= 0.0:
                if elapsed < -self.odom_reset_threshold:
                    self.get_logger().warning(
                        f'丢弃时间戳倒退 {-elapsed:.3f} s 的里程计',
                        throttle_duration_sec=5.0)
                return

            distance, turn = self._pose_change(previous, current)
            if not self._recovering:
                if elapsed > self.odom_max_gap:
                    self._begin_recovery(f'里程计中断 {elapsed:.3f} s')
                elif self._exceeds_jump_limit(distance, turn, elapsed):
                    self._begin_recovery(
                        f'里程计跳变 {distance:.3f} m / {math.degrees(turn):.1f}°'
                        f'（{elapsed:.3f} s）')
                    return
            if self._recovering and not self._recovery_confirmed(current, distance, turn):
                return

        self._last_odometry = current
        self.last_odometry_time = time.monotonic()
        self.x_p, self.y_p, self.z_p = current.position
        self.verified_pose_publisher.publish(pos_msg)
        yaw = current.yaw
        ready = self.first_captured and self.second_captured
        self.calibration_publisher.publish(Bool(data=ready))
        if not ready:
            return
        #yaw = math.degrees(yaw)

        if self.x1==self.x2 and self.y1==self.y2:
            self.x_trans, self.y_trans = self.x_p, self.y_p
            yaw_offset = 0
        else:    
            L = math.sqrt((self.x2-self.x1)**2 + (self.y2-self.y1)**2)
            self.x_trans, self.y_trans,yaw_offset = self.Param_Trans(self.x1, self.y1, self.x2, self.y2, self.x_p, self.y_p, L)
        # self.get_logger().info(f"{x_trans},{y_trans},yaw:{yaw},yaw_offset:{yaw_offset}\n")
        yaw = yaw - yaw_offset
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
        yaw = math.degrees(yaw)
        msg = Float32MultiArray()
        msg.data = [float(self.x_trans), float(self.y_trans), float(yaw)]
        self.publisher.publish(msg)
        #self.x_p = x_trans
        #self.y_p = y_trans
    def Param_Trans(self, x1, y1, x2, y2, x_p, y_p, L):

        x_trans = ((x_p - x1)*(x2 - x1) + (y_p - y1)*(y2 - y1))/L
        y_trans = ((x_p - x1)*(y2 - y1) - (y_p - y1)*(x2 - x1))/L
        yaw_offset = math.atan2((y2 - y1), (x2 - x1))

        return x_trans,y_trans,yaw_offset
        
        

def main(args=None):
    rclpy.init(args=args)
    node = TopicRenamer()
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
