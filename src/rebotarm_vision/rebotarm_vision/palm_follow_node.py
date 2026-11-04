"""Relative, position-only palm following for the DM arm."""
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
from rebotarm_msgs.msg import JointPosVelCmd


class PalmFollowNode(Node):
    def __init__(self):
        super().__init__('rebotarm_palm_follow')
        defaults = dict(arm_namespace='rebotarm', rate_hz=30.0,
                        timeout_sec=0.25, max_joint_speed=0.3,
                        max_tcp_speed=0.04, horizontal_span=0.20,
                        vertical_span=0.16, max_displacement=0.12,
                        smoothing=0.2, deadband=0.008, acquire_frames=5)
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
        self.positions = {}
        self.joint_times = {}
        self.palm_time = -math.inf
        self.anchor = None
        self.count = 0
        self.command = None
        self.last_tick = time.monotonic()
        self.pubs = [self.create_publisher(JointPosVelCmd, f'/{ns}/joints/{n}/cmd/pos_vel', 1) for n in self.names]
        self.create_subscription(JointState, f'/{ns}/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(PointStamped, f'/{ns}/palm_position', self.on_palm, 1)
        self.create_timer(1 / self.cfg['rate_hz'], self.tick)
        self.get_logger().info('DM 手掌跟随就绪：单手张开保持5帧开始，握拳/双手/丢失暂停；请勿同时运行其他运动控制节点')

    def reset(self):
        self.anchor = None
        self.count = 0
        self.command = None

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
        x, y = msg.point.x, msg.point.y
        if (msg.point.z != 1.0 or not 0 <= x <= 1 or not 0 <= y <= 1
                or not 0 <= age < self.cfg['timeout_sec'] or not self.fresh(now)):
            self.reset()
            return
        if now - self.palm_time >= self.cfg['timeout_sec']:
            self.reset()
        self.palm_time = now
        self.count += 1
        if self.count < self.cfg['acquire_frames']:
            return
        uv = np.array([x, y])
        if self.anchor is None:
            self.anchor = uv
            self.origin = self.fk(self.measured_q())
            self.filtered = self.origin.copy()
        delta = uv - self.anchor
        delta[np.abs(delta) < self.cfg['deadband']] = 0
        # Mirrored camera: screen right -> base -Y; screen up -> base +Z.
        offset = np.array([0., -delta[0] * self.cfg['horizontal_span'],
                           -delta[1] * self.cfg['vertical_span']])
        length = np.linalg.norm(offset)
        offset *= min(1., self.cfg['max_displacement'] / max(length, 1e-9))
        self.filtered += self.cfg['smoothing'] * (self.origin + offset - self.filtered)

    def tick(self):
        now = time.monotonic()
        dt = min(now - self.last_tick, 1 / self.cfg['rate_hz'])
        self.last_tick = now
        if not self.fresh(now) or now - self.palm_time >= self.cfg['timeout_sec']:
            self.reset()
        if self.anchor is None:
            return
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
