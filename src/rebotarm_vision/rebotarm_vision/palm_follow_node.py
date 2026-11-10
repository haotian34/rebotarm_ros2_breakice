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
        defaults = dict(arm_namespace='rebotarm', rate_hz=30.0,
                        timeout_sec=0.4, max_joint_speed=0.8,
                        max_tcp_speed=0.18, follow_gain=0.3,
                        chase_gain_scale=0.6, chase_timeout=2.5,
                        workspace_dy=0.25, workspace_z_up=0.18,
                        workspace_z_down=0.12, goto_home_duration=3.0,
                        smoothing=0.5, deadband=0.02, acquire_frames=5)
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
        self.get_logger().info('DM 手掌跟随就绪：启动后自动移动到起手位；张手5帧跟随；'
                               '握拳=暂停；手出画=短暂追丢后停；'
                               '复位=ros2 topic pub --once /%s/palm_follow/reset std_msgs/msg/Bool "{data: true}"' % ns)

    def reset(self):
        self.anchor = None
        self.count = 0
        self.command = None
        self.uv = None
        self.uv_s = None
        self.paused = False
        self.lost_since = None
        self.homing = False

    def on_reset(self, msg):
        if not msg.data or self.phase != 'ready':
            return
        if not self.fresh(time.monotonic()):
            self.get_logger().warn('关节反馈失效，忽略复位指令')
            return
        self.homing = True
        self.paused = False
        self.lost_since = None
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
                                           vlim=self.cfg['max_joint_speed'],
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

        if self.homing:
            self.filtered = self.origin.copy()
        elif self.anchor is None:
            return
        elif not self.paused and stream_alive:
            chasing = self.lost_since is not None
            if chasing and now - self.lost_since > self.cfg['chase_timeout']:
                pass  # 追丢超时：原地保持
            else:
                if chasing:
                    # 手出镜头：沿最后偏离方向降速追
                    err = self.last_err.copy()
                    scale = self.cfg['chase_gain_scale']
                else:
                    # 速度伺服：偏离中心越多目标移动越快，死区内停
                    err = self.uv_s - np.array([0.5, 0.5])
                    scale = 1.0
                    if np.linalg.norm(err) > self.cfg['deadband']:
                        self.last_err = err.copy()
                    err[np.abs(err) < self.cfg['deadband']] = 0.0
                # 镜像画面：屏幕右 = 基座 +Y，上 = +Z
                v = np.array([0.0,
                              self.cfg['follow_gain'] * err[0],
                              -self.cfg['follow_gain'] * err[1]]) * scale
                self.filtered = self.filtered + v * dt
                # 工作空间盒限制（固定基准=起手位）：左右 ±dy，上 +z_up，下 -z_down
                travel = self.filtered - self.origin
                travel[0] = 0.0
                travel[1] = float(np.clip(travel[1], -self.cfg['workspace_dy'],
                                          self.cfg['workspace_dy']))
                travel[2] = float(np.clip(travel[2], -self.cfg['workspace_z_down'],
                                          self.cfg['workspace_z_up']))
                self.filtered = self.origin + travel

        measured = self.measured_q()
        q = measured.copy() if self.command is None else self.command.copy()
        # Stop and reacquire if the driver is not following our commands.
        if np.max(np.abs(q[self.qidx] - measured[self.qidx])) > 0.12:
            self.reset()
            return
        pos = self.fk(q)
        error = self.filtered - pos
        error *= min(1., self.cfg['max_tcp_speed'] * dt / max(np.linalg.norm(error), 1e-9))
        jac = pin.computeFrameJacobian(self.model, self.data, q, self.fid,
                                       pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, self.vidx]
        dq = jac.T @ np.linalg.solve(jac @ jac.T + 1e-4 * np.eye(3), error)
        step = self.cfg['max_joint_speed'] * dt
        dq *= min(1., step / max(np.max(np.abs(dq)), 1e-9))
        candidate = q.copy()
        candidate[self.qidx] += dq
        lo = self.model.lowerPositionLimit[self.qidx]
        hi = self.model.upperPositionLimit[self.qidx]
        if (not np.all(np.isfinite(candidate)) or
                np.any(candidate[self.qidx] < lo) or np.any(candidate[self.qidx] > hi)):
            self.reset()
            return
        self.command = candidate
        for pub, value in zip(self.pubs, candidate[self.qidx]):
            pub.publish(JointPosVelCmd(pos=float(value), vlim=self.cfg['max_joint_speed'],
                                      stamp=self.get_clock().now().to_msg()))
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
