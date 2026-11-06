from __future__ import annotations

import threading

from rclpy.qos import QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import (
    JointMitCmd,
    JointPosVelCmd,
)


class MotorPassthrough:
    def __init__(self, node, hardware, namespace: str, arbitration: str) -> None:
        self._node = node
        self._hardware = hardware
        self._arbitration = arbitration
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._subscriptions = []

        joint_commands = (
            (
                JointMitCmd,
                "cmd/mit",
                lambda hw, name, msg: hw.send_joint_mit_cmd(
                    name,
                    msg.pos,
                    msg.vel,
                    msg.kp,
                    msg.kd,
                    msg.tau,
                ),
            ),
            (
                JointPosVelCmd,
                "cmd/pos_vel",
                lambda hw, name, msg: hw.send_joint_pos_vel_cmd(
                    name,
                    msg.pos,
                    msg.vlim,
                ),
            ),
        )
        gripper_commands = (
            (
                JointMitCmd,
                "cmd/mit",
                lambda hw, msg: hw.send_gripper_mit_cmd(
                    msg.pos,
                    msg.vel,
                    msg.kp,
                    msg.kd,
                    msg.tau,
                ),
            ),
            (
                JointPosVelCmd,
                "cmd/pos_vel",
                lambda hw, msg: hw.send_gripper_pos_vel_cmd(msg.pos, msg.vlim),
            ),
        )

        # Coalesce high-rate arm pos_vel commands: keep only the latest
        # target per joint and flush one batched group send at a fixed rate.
        # Forwarding every message directly used to cost a full-arm feedback
        # read plus a full-arm send per message, saturating the serial CAN
        # bridge and causing jerky motion on real hardware.
        self._pv_lock = threading.Lock()
        self._pv_latest_pos = {}
        self._pv_latest_vlim = {}
        self._pv_dirty = False
        self._pv_timer = self._node.create_timer(
            0.02, self._flush_pos_vel, callback_group=self._node.reentrant_group
        )

        for joint_name in hardware.joint_names:
            for msg_type, label, command in joint_commands:
                if label == "cmd/pos_vel":
                    callback = self._make_joint_pos_vel_callback(joint_name)
                else:
                    callback = self._make_joint_callback(
                        joint_name,
                        label,
                        command,
                    )
                self._subscribe(
                    msg_type,
                    f"/{namespace}/joints/{joint_name}/{label}",
                    callback,
                    qos,
                )
        if hardware.has_gripper:
            for msg_type, label, command in gripper_commands:
                self._subscribe(
                    msg_type,
                    f"/{namespace}/gripper/{label}",
                    self._make_gripper_callback(label, command),
                    qos,
                )

    def _subscribe(self, msg_type, topic: str, callback, qos: QoSProfile) -> None:
        self._subscriptions.append(
            self._node.create_subscription(
                msg_type,
                topic,
                callback,
                qos,
                callback_group=self._node.reentrant_group,
            )
        )

    def _make_joint_pos_vel_callback(self, joint_name: str) -> object:
        def _callback(msg) -> None:
            if not self._can_send_lowlevel(
                f"/joints/{joint_name}/cmd/pos_vel",
                allow_preempt=True,
            ):
                return
            with self._pv_lock:
                self._pv_latest_pos[joint_name] = float(msg.pos)
                self._pv_latest_vlim[joint_name] = float(msg.vlim)
                self._pv_dirty = True

        return _callback

    def _flush_pos_vel(self) -> None:
        with self._pv_lock:
            if not self._pv_dirty:
                return
            positions = dict(self._pv_latest_pos)
            vlims = dict(self._pv_latest_vlim)
            self._pv_dirty = False
        if not self._can_send_lowlevel(
            "/joints/*/cmd/pos_vel(batched)", allow_preempt=True
        ):
            return
        try:
            self._hardware.send_arm_pos_vel_stream(positions, vlims)
        except Exception as exc:
            self._node.get_logger().warn(f"pos_vel stream failed: {exc}")
        finally:
            self._node.publish_arm_status()

    def _make_joint_callback(self, joint_name: str, label: str, command) -> object:
        def _callback(msg) -> None:
            if not self._can_send_lowlevel(
                f"/joints/{joint_name}/{label}",
                allow_preempt=True,
            ):
                return

            try:
                command(self._hardware, joint_name, msg)
            except Exception as exc:
                self._node.get_logger().warn(
                    f"joint {label} failed for {joint_name}: {exc}"
                )
            finally:
                self._node.publish_arm_status()

        return _callback

    def _make_gripper_callback(self, label: str, command) -> object:
        def _callback(msg) -> None:
            if not self._can_send_lowlevel(
                f"/gripper/{label}",
                allow_preempt=False,
            ):
                return

            try:
                command(self._hardware, msg)
            except Exception as exc:
                self._node.get_logger().warn(f"gripper {label} failed: {exc}")
            finally:
                self._node.publish_arm_status()

        return _callback

    def _can_send_lowlevel(self, label: str, *, allow_preempt: bool) -> bool:
        state = self._hardware.state_machine
        if state in ("GRAVITY_COMP", "SAFE_HOMING"):
            self._node.get_logger().warn(f"rejecting {label} in state {state}")
            return False
        if state == "TRAJ_RUNNING":
            if self._arbitration == "reject" or not allow_preempt:
                self._node.get_logger().warn(
                    f"rejecting {label} while trajectory is running"
                )
                return False
            self._node.get_logger().warn(
                f"preempting trajectory for {label}"
            )
            self._hardware.stop_motion()
        return True
