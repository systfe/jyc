"""
小车纯跟踪控制模块。

使用方式示例：

    import rclpy
    from DriveControl import DriveControl, DriveControlConfig

    rclpy.init()
    drive = DriveControl()

    drive.move_to_abs(1.0, 0.5)       # 移动到地图坐标 x=1.0m, y=0.5m
    drive.turn_to_abs(90.0)           # 原地转到地图坐标系 90 度
    drive.move_to_relative(0.3, 0.0)  # 相对车体向前 0.3m
    drive.turn_by(-45.0)              # 相对当前朝向右转 45 度
    pose = drive.print_pose()         # 打印并返回当前位置

    drive.destroy_node()
    rclpy.shutdown()

坐标约定：
    - 订阅定位：/robot/pose，类型 geometry_msgs/msg/PoseStamped。
    - 发布速度：/robot/cmd_vel，类型 geometry_msgs/msg/Twist。
    - 位置单位：米。
    - 角度单位：外部接口使用“度”，内部计算使用“弧度”。
    - move_to_relative(dx, dy) 默认使用车体坐标：
        dx > 0 表示向车头方向移动；
        dy > 0 表示向车体左侧移动；
        dy < 0 表示向车体右侧移动。
"""

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node


@dataclass
class DriveControlConfig:
    """控制参数集中放在这里，实车调参时优先改这一块。"""

    # 话题名。仿真和实物都建议通过 robot_bringup 统一成 /robot/*。
    pose_topic: str = "/robot/pose"
    cmd_vel_topic: str = "/robot/cmd_vel"

    # 控制循环频率。越高越平滑，但 CPU 占用更高；一般 20~50Hz 足够。
    control_hz: float = 30.0

    # 到点判定阈值。小于这个距离就认为已经到达目标点。
    position_tolerance_m: float = 0.008

    # 角度判定阈值。小于这个角度误差就认为朝向已经到达。
    yaw_tolerance_deg: float = 1.0

    # 平移 P 控制系数。越大响应越快，但过大容易抖动或冲过目标。
    linear_kp: float = 1.2

    # 角速度 P 控制系数。越大转向越快，但过大容易来回摆。
    angular_kp: float = 1.5

    # 最大线速度，单位 m/s。实车初期建议保守一些。
    max_linear_speed: float = 0.5

    # 最小线速度，单位 m/s。距离很小时若速度过小，小车可能因为摩擦不动。
    # 设置为 0 表示完全按 P 控制输出；实车可尝试 0.03~0.08。
    min_linear_speed: float = 0.02

    # 最大角速度，单位 deg/s。外部按度调参更直观，内部会自动转成 rad/s。
    max_angular_speed_deg: float = 30.0

    # 最小角速度，单位 deg/s。实车原地转时如果起不来，可适当增大。
    min_angular_speed_deg: float = 6.0

    # 移动到点时是否同时修正朝向。
    # False：只管 x/y，到点后不管车头朝哪里。
    # True：移动过程中按 target_yaw_deg 修正车头。
    # 注意：Move() 显式传入 target_yaw_deg 时会优先修正朝向。
    rotate_while_moving: bool = False

    # 如果定位话题长时间没有数据，等待多久后报错。
    pose_wait_timeout_s: float = 5.0

    # 单次动作默认超时时间，避免目标不可达时一直卡住。
    default_action_timeout_s: float = 20.0

    # 停车后额外发布几帧 0 速度，让底盘更稳地停下来。
    stop_publish_count: int = 5


class DriveControl(Node):
    """面向外部控制逻辑的简单跟踪控制器。"""

    def __init__(self, config: Optional[DriveControlConfig] = None):
        super().__init__("drive_control")
        self.config = config or DriveControlConfig()

        self._pose: Optional[Tuple[float, float, float]] = None
        self._last_pose_time: Optional[float] = None

        self._pose_sub = self.create_subscription(
            PoseStamped,
            self.config.pose_topic,
            self._pose_callback,
            10,
        )
        self._cmd_pub = self.create_publisher(
            Twist,
            self.config.cmd_vel_topic,
            10,
        )

        self.get_logger().info(
            f"DriveControl 已启动: pose={self.config.pose_topic}, "
            f"cmd_vel={self.config.cmd_vel_topic}"
        )

    # -------------------------------------------------------------------------
    # 外部建议调用的函数
    # -------------------------------------------------------------------------

    def Move(
        self,
        x: float,
        y: float,
        target_yaw_deg: Optional[float] = None,
        timeout_s: Optional[float] = None,
    ) -> bool:
        """
        移动到地图绝对坐标。

        参数：
            x, y：目标地图坐标，单位 m。
            target_yaw_deg：可选目标朝向，单位 deg。
                - 若 config.rotate_while_moving=False，则移动时不修正朝向。
                - 若 config.rotate_while_moving=True 且提供该参数，则边走边修正朝向。
            timeout_s：本次动作超时时间；不填则使用默认值。

        返回：
            True：到达目标。
            False：超时或没有定位。
        """
        if not self.wait_for_pose():
            return False

        timeout_s = self._resolve_timeout(timeout_s)
        target_yaw_rad = None if target_yaw_deg is None else math.radians(target_yaw_deg)
        start_time = time.monotonic()
        period = 1.0 / self.config.control_hz

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            pose = self.get_pose()
            if pose is None:
                self.stop()
                return False

            cur_x, cur_y, cur_yaw = pose
            dx = x - cur_x
            dy = y - cur_y
            distance = math.hypot(dx, dy)

            if distance <= self.config.position_tolerance_m:
                self.stop()
                return True

            if time.monotonic() - start_time > timeout_s:
                self.get_logger().warn("move_to_abs 超时，已停车")
                self.stop()
                return False

            cmd = Twist()

            # 把地图坐标误差转到车体坐标，这样 cmd.linear.x/y 就能直接表达
            # “车头方向”和“车体左侧方向”的速度。
            body_x, body_y = self._world_vector_to_body(dx, dy, cur_yaw)
            linear_speed = self._clamp_with_min(
                self.config.linear_kp * distance,
                self.config.min_linear_speed,
                self.config.max_linear_speed,
            )
            cmd.linear.x = linear_speed * body_x / max(distance, 1e-6)
            cmd.linear.y = linear_speed * body_y / max(distance, 1e-6)

            if target_yaw_rad is not None or self.config.rotate_while_moving:
                if target_yaw_rad is None:
                    target_yaw_rad = cur_yaw
                yaw_error = self._normalize_angle(target_yaw_rad - cur_yaw)
                cmd.angular.z = self._calc_angular_speed(yaw_error)

            self._cmd_pub.publish(cmd)
            time.sleep(period)

    def Move_relative(
        self,
        dx: float,
        dy: float,
        timeout_s: Optional[float] = None,
    ) -> bool:
        """
        移动到相对位置。

        默认按“当前车体坐标系”理解：
            dx > 0：向前；
            dx < 0：向后；
            dy > 0：向左；
            dy < 0：向右。

        例如：
            move_to_relative(0.5, 0.0)  向前走 0.5m
            move_to_relative(0.0, -0.3) 向右平移 0.3m
        """
        if not self.wait_for_pose():
            return False

        cur_x, cur_y, cur_yaw = self.get_pose()
        world_dx, world_dy = self._body_vector_to_world(dx, dy, cur_yaw)
        return self.Move(cur_x + world_dx, cur_y + world_dy, timeout_s=timeout_s)

    def Turn_to(
        self,
        target_yaw_deg: float,
        timeout_s: Optional[float] = None,
    ) -> bool:
        """
        原地转到地图绝对朝向。

        参数：
            target_yaw_deg：目标朝向，单位 deg。
                0 度通常表示地图 x 正方向，具体取决于 /robot/pose 的坐标定义。
        """
        if not self.wait_for_pose():
            return False

        timeout_s = self._resolve_timeout(timeout_s)
        target_yaw = math.radians(target_yaw_deg)
        start_time = time.monotonic()
        period = 1.0 / self.config.control_hz
        tolerance = math.radians(self.config.yaw_tolerance_deg)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            pose = self.get_pose()
            if pose is None:
                self.stop()
                return False

            _, _, cur_yaw = pose
            yaw_error = self._normalize_angle(target_yaw - cur_yaw)

            if abs(yaw_error) <= tolerance:
                self.stop()
                return True

            if time.monotonic() - start_time > timeout_s:
                self.get_logger().warn("turn_to_abs 超时，已停车")
                self.stop()
                return False

            cmd = Twist()
            cmd.angular.z = self._calc_angular_speed(yaw_error)
            self._cmd_pub.publish(cmd)
            time.sleep(period)

    def Turn_by(
        self,
        delta_yaw_deg: float,
        timeout_s: Optional[float] = None,
    ) -> bool:
        """
        相对当前朝向转动指定角度。

        参数：
            delta_yaw_deg：相对转角，单位 deg。
                正数：逆时针转；
                负数：顺时针转。
        """
        if not self.wait_for_pose():
            return False

        _, _, cur_yaw = self.get_pose()
        target_yaw = self._normalize_angle(cur_yaw + math.radians(delta_yaw_deg))
        return self.Turn_to(math.degrees(target_yaw), timeout_s=timeout_s)

    def Print_pose(self) -> Optional[Tuple[float, float, float]]:
        """
        打印并返回当前绝对坐标。

        返回：
            (x, y, yaw_deg) 或 None。
        """
        rclpy.spin_once(self, timeout_sec=0.0)
        pose = self.get_pose()
        if pose is None:
            self.get_logger().warn("当前还没有收到 /robot/pose")
            return None

        x, y, yaw = pose
        yaw_deg = math.degrees(yaw)
        self.get_logger().info(f"当前坐标: x={x:.3f}m, y={y:.3f}m, yaw={yaw_deg:.1f}deg")
        return x, y, yaw_deg

    # -------------------------------------------------------------------------
    # 内部工具函数
    # -------------------------------------------------------------------------

    def get_pose(self) -> Optional[Tuple[float, float, float]]:
        """返回当前位姿，格式为 (x, y, yaw_rad)。"""
        return self._pose

    def wait_for_pose(self) -> bool:
        """等待第一帧定位数据，避免刚启动时控制器没有坐标就开始发速度。"""
        start_time = time.monotonic()
        while rclpy.ok() and self._pose is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - start_time > self.config.pose_wait_timeout_s:
                self.get_logger().error("等待 /robot/pose 超时")
                return False
        return True

    def stop(self) -> None:
        """发布 0 速度停车。多发几帧可以降低底盘漏收一帧造成的风险。"""
        cmd = Twist()
        for _ in range(max(1, self.config.stop_publish_count)):
            self._cmd_pub.publish(cmd)
            time.sleep(0.01)

    def _pose_callback(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)
        self._pose = (msg.pose.position.x, msg.pose.position.y, yaw)
        self._last_pose_time = time.monotonic()

    def _resolve_timeout(self, timeout_s: Optional[float]) -> float:
        return self.config.default_action_timeout_s if timeout_s is None else timeout_s

    def _calc_angular_speed(self, yaw_error: float) -> float:
        angular_speed = self.config.angular_kp * yaw_error
        max_w = math.radians(self.config.max_angular_speed_deg)
        min_w = math.radians(self.config.min_angular_speed_deg)
        return self._clamp_with_min(angular_speed, min_w, max_w)

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """四元数转 yaw。只取平面小车需要的 z 轴朝向。"""
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """把角度规整到 [-pi, pi]，确保小车走最短转向路径。"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _clamp_with_min(value: float, min_abs: float, max_abs: float) -> float:
        """
        限幅并保留最小输出。

        例如 value 很小但非零时，输出至少 min_abs，避免实车因为摩擦不动。
        如果 min_abs 设置为 0，就退化成普通限幅。
        """
        if abs(value) < 1e-9:
            return 0.0

        sign = 1.0 if value > 0.0 else -1.0
        limited = min(abs(value), max_abs)
        if min_abs > 0.0:
            limited = max(limited, min_abs)
        return sign * limited

    @staticmethod
    def _world_vector_to_body(dx: float, dy: float, yaw: float) -> Tuple[float, float]:
        """
        地图坐标向量转车体坐标向量。

        车体坐标：
            x 轴：车头方向；
            y 轴：车体左侧方向。
        """
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        body_x = cos_yaw * dx + sin_yaw * dy
        body_y = -sin_yaw * dx + cos_yaw * dy
        return body_x, body_y

    @staticmethod
    def _body_vector_to_world(dx: float, dy: float, yaw: float) -> Tuple[float, float]:
        """车体坐标向量转地图坐标向量。"""
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        world_x = cos_yaw * dx - sin_yaw * dy
        world_y = sin_yaw * dx + cos_yaw * dy
        return world_x, world_y


