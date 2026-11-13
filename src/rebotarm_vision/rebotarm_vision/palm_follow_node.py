"""Relative, position-only palm following for the DM arm.

Eye-in-hand servo (camera mounted on the arm):
  - 启动后自动平滑移动到「鸡脖子」起手位，无需手动拉起机械臂
  - 手偏离画面中心 -> 末端持续向该方向运动，直到手回到中心（死区内停止）
  - 手出镜头 -> 沿最后的方向短暂追丢（降速、限时），到工作空间边界即停
  - 手在画面中但未张开（握拳/双手）-> 暂停保持
  - 工作空间以起手位为固定基准，与张手位置无关
  - /{ns}/palm_follow/reset (Bool) -> 平滑回到起手位并等待重新张手
"""
import math
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from rebotarm_msgs.msg import JointPosVelCmd
from rebotarm_vision.follow_control import cartesian_lag_scale, joint_lag_scale

# 「鸡脖子」悬空起手位（与 greeting/dance 的 home_pose 一致）
HOME_Q = [0.0, -0.62, -0.85, 0.0, 0.0, 0.0]


class PalmFollowNode(Node):
    def __init__(self):
        super().__init__('rebotarm_palm_follow')
        # 眼在手上（摄像头随臂动）速度伺服参数：
        #   follow_gain       满幅偏离时目标点移动速度（m/s）
        #   chase_gain_scale  手出镜头后追丢速度 = follow_gain × 该系数
        #   chase_timeout     追丢最长持续时间（秒），超时原地保持
        #   workspace_dy      左右半程（米，相对起手位）
        #   workspace_z_up/down 上/下半程（米）
        #   goto_home_duration 启动时移动到起手位的时长（秒）
        # 工作空间是以起手位（鸡脖子位，约0.46m高）为中心的固定盒子：
        # ±dy、+z_up/-z_down，X 固定；下边界 0.34m 保证碰不到台面。
        defaults = dict(arm_namespace='rebotarm', rate_hz=100.0,
                        timeout_sec=0.4, max_joint_speed=5.0,
                        max_tcp_speed=0.6, follow_gain=1.65,
                        max_joint_accel=12.0, max_tcp_accel=2.0, braking_gain=1.5,
                        max_target_lead=0.06, home_joint_speed=0.8,
                        chase_gain_scale=0.1090909091, chase_timeout=2.5,
                        workspace_dy=0.25, workspace_z_up=0.18,
                        workspace_z_down=0.12, goto_home_duration=3.0,
                        smoothing=0.85, deadband=0.02, acquire_frames=3)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        for name in defaults.keys() - {'arm_namespace'}:
            if not math.isfinite(self.cfg[name]) or self.cfg[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.cfg['smoothing'] > 1:
            raise ValueError('smoothing must be <= 1')
        ns = self.cfg['arm_namespace'].strip('/')
        urdf = Path(get_package_share_directory('rebotarm_bringup')) / 'description/DM/urdf/ReBot_Arm_DM.urdf'
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.data = self.model.createData()
        self.fid = self.model.getFrameId('end_link')
        self.names = [f'joint{i}' for i in range(1, 7)]
        self.qidx = [self.model.joints[self.model.getJointId(n)].idx_q for n in self.names]
        self.vidx = [self.model.joints[self.model.getJointId(n)].idx_v for n in self.names]
        # DM SDK的POS_VEL配置；并非电机在任意负载下的性能保证。
        self.joint_speed_limits = np.minimum(
            self.cfg['max_joint_speed'], [5.0, 5.0, 5.0, 3.0, 3.0, 3.0])
        self.joint_velocity = np.zeros(6)
        self.tcp_velocity = np.zeros(3)
        # 工作空间基准：固定为起手位的 TCP，与张手位置无关
        q_home = pin.neutral(self.model)
        q_home[self.qidx] = HOME_Q
        self.origin = self.fk(q_home)
        self.positions = {}
        self.joint_times = {}
        self.palm_time = -math.inf      # 最近一条手掌消息（任意状态）
        self.uv_valid_time = -math.inf  # 最近一次有效张开手掌
        self.phase = 'init'             # init / goto_home / ready
        self.home_from = None
        self.home_t = 0.0
        self.anchor = None              # True = 已建立跟随基准
        self.count = 0
        self.command = None
        self.uv = None                  # 原始手心图像坐标
        self.uv_s = None                # 低通后的手心图像坐标
        self.filtered = None            # 末端目标点
        self.last_err = np.zeros(2)     # 最后一次有效偏离方向（追丢用）
        self.paused = False             # 手在画面中但未张开
        self.lost_since = None          # 手出镜头的时间点
        self.homing = False             # 正在复位回起手位
        self.last_tick = time.monotonic()
        self.pubs = [self.create_publisher(JointPosVelCmd, f'/{ns}/joints/{n}/cmd/pos_vel', 1) for n in self.names]
        self.create_subscription(JointState, f'/{ns}/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(PointStamped, f'/{ns}/palm_position', self.on_palm, 1)
        self.create_subscription(Bool, f'/{ns}/palm_follow/reset', self.on_reset, 1)
        self.create_timer(1 / self.cfg['rate_hz'], self.tick)
        self.get_logger().info('DM 手掌跟随就绪：启动后自动移动到起手位；张手%d帧跟随；'
                               '握拳=暂停；手出画=短暂追丢后停；'
                               '复位=ros2 topic pub --once /%s/palm_follow/reset std_msgs/msg/Bool "{data: true}"' % (self.cfg['acquire_frames'], ns))
        self.get_logger().info(
            f"跟随配置：{self.cfg['rate_hz']:g}Hz，gain={self.cfg['follow_gain']:g}，"
            f"TCP≤{self.cfg['max_tcp_speed']:g}m/s，关节上限={self.joint_speed_limits.tolist()}")

    def reset(self):
        self.anchor = None
        self.count = 0
        self.command = None
        self.uv = None
        self.uv_s = None
        self.paused = False
        self.lost_since = None
        self.homing = False
        self.joint_velocity[:] = 0.0
        self.tcp_velocity[:] = 0.0

    def on_reset(self, msg):
        if not msg.data or self.phase != 'ready':
            return
        if not self.fresh(time.monotonic()):
            self.get_logger().warn('关节反馈失效，忽略复位指令')
            return
        self.homing = True
        self.paused = False
        self.lost_since = None
        self.joint_velocity[:] = 0.0
        self.tcp_velocity[:] = 0.0
        self.get_logger().info('收到复位指令：平滑回到起手位')

    def on_joints(self, msg):
        now = time.monotonic()
        for name, value in zip(msg.name, msg.position):
            if name in self.names and math.isfinite(value):
                self.positions[name] = value
                self.joint_times[name] = now

    def fresh(self, now):
        return all(now - self.joint_times.get(n, -math.inf) < self.cfg['timeout_sec'] for n in self.names)

    def measured_q(self):
        q = pin.neutral(self.model)
        q[self.qidx] = [self.positions[n] for n in self.names]
        return q

    def fk(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.fid].translation.copy()

    def on_palm(self, msg):
        now = time.monotonic()
        age = (self.get_clock().now().nanoseconds * 1e-9 -
               (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))
        if not 0 <= age < self.cfg['timeout_sec']:
            return  # 过期消息直接忽略，不破坏当前状态
        z = msg.point.z
        if z == 0.5:
            # 手在画面中但未张开：暂停保持，保留基准
            self.paused = True
            self.lost_since = None
            self.palm_time = now
            return
        if z != 1.0 or not (0 <= msg.point.x <= 1 and 0 <= msg.point.y <= 1):
            # 画面中无手：进入追丢模式（仅已建立基准时有意义）
            if self.anchor is not None:
                if self.lost_since is None:
                    self.lost_since = now
                self.paused = False
                self.palm_time = now
            return
        # ---- 有效张开手掌 ----
        if not self.fresh(now):
            return
        self.paused = False
        self.lost_since = None
        self.palm_time = now
        uv = np.array([msg.point.x, msg.point.y])
        if now - self.uv_valid_time > 0.8:
            # 长时间脱手后重新出现：直接吸附，避免滤波滑移造成反向抽动
            self.uv_s = uv
        self.uv_valid_time = now
        self.count += 1
        if self.count < self.cfg['acquire_frames']:
            return
        if self.anchor is None:
            self.anchor = True
            # 目标点从当前实际位置起步；工作空间基准固定为起手位
            self.filtered = self.fk(self.measured_q())
            self.uv_s = uv
        # 对手心图像坐标做低通，抑制 MediaPipe 抖动
        self.uv_s += self.cfg['smoothing'] * (uv - self.uv_s)
        self.uv = uv

    def tick(self):
        now = time.monotonic()
        # 允许最多 2 倍节拍的补偿步长，偶发卡顿的 tick 能追上，不累积滞后
        dt = min(now - self.last_tick, 2.0 / self.cfg['rate_hz'])
        self.last_tick = now
        if dt <= 0.0:
            return

        # ---- 启动阶段：等反馈就绪，然后平滑移动到起手位 ----
        if self.phase == 'init':
            if not self.fresh(now):
                return
            self.home_from = [self.positions[n] for n in self.names]
            self.home_t = 0.0
            self.phase = 'goto_home'
            self.get_logger().info('正在移动到起手位（鸡脖子）…')
            return
        if self.phase == 'goto_home':
            if not self.fresh(now):
                return
            self.home_t += dt
            u = min(1.0, self.home_t / self.cfg['goto_home_duration'])
            eased = 0.5 - 0.5 * math.cos(math.pi * u)
            stamp = self.get_clock().now().to_msg()
            for pub, s, h in zip(self.pubs, self.home_from, HOME_Q):
                pub.publish(JointPosVelCmd(pos=float(s + (h - s) * eased),
                                           vlim=self.cfg['home_joint_speed'],
                                           stamp=stamp))
            if u >= 1.0:
                self.phase = 'ready'
                self.filtered = self.origin.copy()
                self.get_logger().info('已到起手位，张开手掌开始跟随')
            return

        if not self.fresh(now):
            self.reset()
            return
        stream_alive = now - self.palm_time < self.cfg['timeout_sec']
        # 不追赶暂停/过期视觉数据积累的旧目标；只可能完成上一个小步。
        chase_expired = (self.lost_since is not None and
                         now - self.lost_since > self.cfg['chase_timeout'])
        if not self.homing and (self.paused or not stream_alive or chase_expired):
            self.filtered = self.fk(self.measured_q())
            self.joint_velocity[:] = 0.0
            self.tcp_velocity[:] = 0.0
            return

        if not self.homing and self.anchor is None:
            return
        measured = self.measured_q()
        q = measured.copy() if self.command is None else self.command.copy()
        lag = float(np.max(np.abs(q[self.qidx] - measured[self.qidx])))
        if lag > 0.12:
            self.get_logger().warn(f'跟踪误差{lag:.3f}rad，等待重新张手', throttle_duration_sec=2.0)
            self.reset()
            return
        pos = self.fk(q)
        if self.homing:
            desired_velocity = 4.0 * (self.origin - pos)
        else:
            chasing = self.lost_since is not None
            err = self.last_err.copy() if chasing else self.uv_s - np.array([0.5, 0.5])
            if not chasing:
                self.last_err = err.copy()
            # 连续死区：过阈值后从零开始，避免中心附近速度跳变。
            err = np.sign(err) * np.maximum(np.abs(err) - self.cfg['deadband'], 0.0)
            scale = self.cfg['chase_gain_scale'] if chasing else 1.0
            desired_velocity = np.array([0.0, err[0], -err[1]]) * self.cfg['follow_gain'] * scale
            desired_velocity[0] = 4.0 * (self.origin[0] - pos[0])
        # 直接从图像误差得到速度，不把位置误差除以dt放大，也不积累追赶目标。
        tcp_speed = min(self.cfg['max_tcp_speed'], 0.18) if self.homing else self.cfg['max_tcp_speed']
        desired_velocity *= min(1., tcp_speed / max(np.linalg.norm(desired_velocity), 1e-9))
        # 在硬停止阈值之前连续降速，跟踪慢的执行器不再反复触发复位。
        desired_velocity *= cartesian_lag_scale(
            pos - self.fk(measured), desired_velocity, self.cfg['max_target_lead'])
        lower = self.origin + np.array([-np.inf, -self.cfg['workspace_dy'], -self.cfg['workspace_z_down']])
        upper = self.origin + np.array([np.inf, self.cfg['workspace_dy'], self.cfg['workspace_z_up']])
        for axis in (1, 2):
            # 留一个控制周期的制动余量，在边界前平滑减速。
            distance = upper[axis] - pos[axis] if desired_velocity[axis] >= 0 else pos[axis] - lower[axis]
            brake_speed = math.sqrt(2.0 * self.cfg['max_tcp_accel'] * max(0.0, distance - tcp_speed * dt))
            desired_velocity[axis] = np.clip(desired_velocity[axis], -brake_speed, brake_speed)
        dv = desired_velocity - self.tcp_velocity
        braking = float(np.dot(desired_velocity, self.tcp_velocity)) < 0.0
        accel = self.cfg['max_tcp_accel'] * (self.cfg['braking_gain'] if braking else 1.0)
        dv *= min(1., accel * dt / max(np.linalg.norm(dv), 1e-9))
        self.tcp_velocity += dv
        error = self.tcp_velocity * dt
        self.filtered = np.clip(pos + error, lower, upper)
        self.filtered[0] = self.origin[0]
        jac = pin.computeFrameJacobian(self.model, self.data, q, self.fid,
                                       pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, self.vidx]
        dq = jac.T @ np.linalg.solve(jac @ jac.T + 1e-4 * np.eye(3), error)
        limits = self.joint_speed_limits
        if self.homing:
            limits = np.minimum(limits, self.cfg['home_joint_speed'])
        desired_joint_velocity = dq / dt
        desired_joint_velocity *= joint_lag_scale(q[self.qidx] - measured[self.qidx],
                                                  desired_joint_velocity)
        desired_joint_velocity *= min(1., 1. / max(
            float(np.max(np.abs(desired_joint_velocity) / limits)), 1e-9))
        delta = self.cfg['max_joint_accel'] * dt
        braking = desired_joint_velocity * self.joint_velocity < 0.0
        delta = np.where(braking, delta * self.cfg['braking_gain'], delta)
        self.joint_velocity += np.clip(desired_joint_velocity - self.joint_velocity, -delta, delta)
        self.joint_velocity = np.clip(self.joint_velocity, -limits, limits)
        dq = self.joint_velocity * dt
        candidate = q.copy()
        candidate[self.qidx] += dq
        lo = self.model.lowerPositionLimit[self.qidx]
        hi = self.model.upperPositionLimit[self.qidx]
        if (not np.all(np.isfinite(candidate)) or
                np.any(candidate[self.qidx] < lo) or np.any(candidate[self.qidx] > hi)):
            self.reset()
            return
        # 高速减速时也检查实际FK位置，不能只钳制目标点后让惯性步越过盒边界。
        def inside_workspace(value):
            travel = self.fk(value) - self.origin
            return (abs(travel[1]) <= self.cfg['workspace_dy'] and
                    -self.cfg['workspace_z_down'] <= travel[2] <= self.cfg['workspace_z_up'])

        if not inside_workspace(candidate):
            for _ in range(16):
                dq *= 0.5
                candidate[self.qidx] = q[self.qidx] + dq
                if inside_workspace(candidate):
                    break
            else:
                self.reset()
                return
            self.joint_velocity[:] = 0.0
            self.tcp_velocity[:] = 0.0
        self.command = candidate
        stamp = self.get_clock().now().to_msg()
        for pub, value, limit, velocity in zip(self.pubs, candidate[self.qidx], limits, self.joint_velocity):
            # 小步位置命令配套其实际速度，避免用5rad/s冲到每个采样点后停等。
            motor_speed = min(float(limit), max(0.1, abs(float(velocity)) * 1.2))
            pub.publish(JointPosVelCmd(pos=float(value), vlim=motor_speed,
                                      stamp=stamp))
        # 复位到位判定：末端回到起手位 1cm 内则完成，等待重新张手
        if self.homing and np.linalg.norm(pos - self.origin) < 0.01:
            self.reset()
            self.get_logger().info('已复位到起手位，等待重新张手')


def main(args=None):
    rclpy.init(args=args)
    node = PalmFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
