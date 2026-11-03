#!/usr/bin/env python3
"""
reBot Arm 任务二：舞蹈轨迹离线预计算

用官方 SDK（Pinocchio）离线生成两段舞蹈的 30Hz 关节轨迹，写入
config/dance_tracks.yaml，运行时节点只做插值回放，不在线解 IK。

  段1「机械街舞」 groove:
      TCP 位置钉死在 home 姿态末端点 (0.2844, 0, 0.4587)，
      j1/j2/j3 按编舞曲线摆动（底座摇摆 + 肩肘起伏），
      用「加权 CLIK」求解——身体关节权重贵、腕部权重便宜，
      求解器优先动腕部补偿，从而保留尽可能大的身体摆幅。
      （实测：官方等权 DLS 会吃掉约 70% 摆幅，加权后 j1 ±0.9
        的输入可保留 ±0.6，TCP 误差 < 0.001mm）

  段2「空间滑步」 slide:
      TCP 走矩形轨迹 左上→左下→右下→右上→左上，两端各加
      从 home 的进/出引导段。用官方 solve_ik 位置-only IK
      （链式初值保证解连续）：腕部留在冗余零空间里最省力的
      位置，夹爪姿态自然保持大致朝前（全程漂移 <21°），
      同时底座扫掠角从固定姿态方案的 ±1.14rad 降到 ±0.54rad，
      关节速度才能压进驱动限速。边角用余弦缓动减速。

每帧做三项校验，任一不达标即报错退出（不写文件）：
  1. FK 位置误差 < MAX_POS_ERR（2mm）
  2. 相邻帧线性插值中点的 FK 漂移 < MAX_POS_ERR（保证节点
     30Hz 回放 / 驱动器内部插值时 TCP 仍达标）
  3. 关节限位余量 > LIMIT_MARGIN，相邻帧关节速度 < MAX_JOINT_SPEED
     （仿真驱动 max_joint_speed=1.0 rad/s，留 10% 余量）

用法：
  python3 -m rebotarm_dance.precompute_dance          # 在源码树中运行
  python3 src/rebotarm_dance/rebotarm_dance/precompute_dance.py
"""

import math
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml

# ---- 定位工作空间与官方 SDK ------------------------------------------------
# 本文件: <ws>/src/rebotarm_dance/rebotarm_dance/precompute_dance.py
SRC_DIR = Path(__file__).resolve().parents[2]          # <ws>/src
WS_DIR = SRC_DIR.parent                                # <ws>
SDK_DIR = WS_DIR / "third_party" / "reBotArm_control_py"
sys.path.insert(0, str(SDK_DIR))

from reBotArm_control_py.kinematics.inverse_kinematics import (  # noqa: E402
    IKParams,
    pos_rot_to_se3,
    solve_ik,
)

URDF = SRC_DIR / "rebotarm_bringup" / "description" / "DM" / "urdf" / "ReBot_Arm_DM.urdf"
OUT_YAML = Path(__file__).resolve().parents[1] / "config" / "dance_tracks.yaml"
DANCE_CFG = Path(__file__).resolve().parents[1] / "config" / "dance.yaml"

# ---- 全局参数 ---------------------------------------------------------------
RATE_HZ = 30.0                 # 轨迹采样率（与 dance_node 回放频率一致）
MAX_POS_ERR = 0.002            # TCP 位置误差上限 2mm
LIMIT_MARGIN = 0.05            # 关节限位最小余量（rad）
MAX_JOINT_SPEED = 0.95         # 相邻帧关节速度上限（驱动限 1.0，留余量）

HOME_Q = np.array([0.0, -0.62, -0.85, 0.0, 0.0, 0.0])   # 悬空鸡脖子
JOINT_LIMITS_LO = np.array([-2.8, -3.14, -3.14, -1.87, -1.57, -3.14])
JOINT_LIMITS_HI = np.array([2.8, 0.0, 0.0, 1.57, 1.57, 3.14])

# ---- 段1「机械街舞」编舞参数 ------------------------------------------------
GROOVE = {
    "loop_period": 7.0,    # 一圈时长（秒）；节点侧用 times=2 跳两圈
    "amp_j1": 0.70,        # 底座摇摆输入幅值（加权求解后保留 ±0.59，已饱和）
    "amp_j2": 0.22,        # 肩部下压幅值（二倍频起伏）
    "amp_j3": 0.20,        # 肘部反向伸展幅值（与肩形成 bob 律动）
    "w_body": 0.02,        # 身体关节(j1/2/3)代价权重：越小越"贵"越不爱动
    "max_iter": 3000,
    "tol": 1e-7,
}

# ---- 段2「空间滑步」编舞参数 ------------------------------------------------
# 矩形四角（x 固定 0.28m，基座前方）；左=y+ 右=y- 上=z0.44 下=z0.33
SLIDE = {
    "x": 0.28,
    "y_half": 0.22,
    "z_top": 0.44,
    "z_bottom": 0.33,
    "lead_in": 1.5,        # home → 左上
    "edge": 2.0,           # 每条边
    "lead_out": 1.5,       # 左上 → home
    "max_iter": 2000,
    "tol": 1e-7,
}


# =============================================================================
# 模型加载
# =============================================================================

def load_dm_model():
    model = pin.buildModelFromUrdf(str(URDF))
    data = model.createData()
    fid = model.getFrameId("end_link")
    return model, data, fid


def fk_pos(model, data, fid, q6):
    """q6(6,) -> end_link 位置（手指被动关节补 0）。"""
    q = np.zeros(model.nq)
    q[:6] = q6
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    return data.oMf[fid].translation.copy()


def home_tcp(model, data, fid):
    q = np.zeros(model.nq)
    q[:6] = HOME_Q
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    return data.oMf[fid].copy()


# =============================================================================
# 段1：加权 CLIK（身体关节贵、腕部便宜）——TCP 钉死，保留身体摆幅
# =============================================================================

def weighted_clik_fixed_tcp(model, data, fid, target_pos, q_init6,
                            w_body, max_iter, tol):
    """位置-only 加权 CLIK。W 小的关节单步位移小（"贵"），求解优先动腕部。"""
    nq = model.nq
    q = np.zeros(nq)
    q[:6] = q_init6
    lo = np.where(np.isfinite(model.lowerPositionLimit),
                  model.lowerPositionLimit, -np.inf)
    hi = np.where(np.isfinite(model.upperPositionLimit),
                  model.upperPositionLimit, np.inf)
    W = np.ones(nq)
    W[0:3] = w_body                      # j1/j2/j3 贵
    W[6:] = 1e-3                         # 被动手指基本锁死

    for _ in range(max_iter):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        pin.computeJointJacobians(model, data, q)
        T = data.oMf[fid]
        err = T.rotation.T @ (target_pos - T.translation)   # LOCAL 系位置误差
        if np.linalg.norm(err) < tol:
            break
        J = pin.getFrameJacobian(model, data, fid, pin.LOCAL)[:3, :]
        dq = W * (J.T @ np.linalg.solve((J * W) @ J.T + 1e-6 * np.eye(3), err))
        # 限位处阻塞继续向外的分量
        blocked = ((q <= lo + 1e-9) & (dq < 0)) | ((q >= hi - 1e-9) & (dq > 0))
        dq[blocked] = 0.0
        q = np.clip(q + 0.5 * dq, lo, hi)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    final_err = float(np.linalg.norm(data.oMf[fid].translation - target_pos))
    return q[:6].copy(), final_err


def groove_ref(phase, p=GROOVE):
    """编舞参考曲线（作为加权 IK 的初值增量），首尾严格等于 home。"""
    j1 = p["amp_j1"] * math.sin(phase)
    j2 = HOME_Q[1] - p["amp_j2"] * (0.5 - 0.5 * math.cos(2.0 * phase))
    j3 = HOME_Q[2] + p["amp_j3"] * (0.5 - 0.5 * math.cos(2.0 * phase))
    return np.array([j1, j2, j3, 0.0, 0.0, 0.0])


def gen_groove(model, data, fid, tcp_target):
    """生成一圈街舞轨迹（首尾都是 home，节点循环 times 次）。"""
    n_frames = int(round(GROOVE["loop_period"] * RATE_HZ))
    frames = []
    q_sol = HOME_Q.copy()
    prev_ref = groove_ref(0.0)
    for i in range(n_frames):
        phase = 2.0 * math.pi * i / n_frames
        ref = groove_ref(phase)
        # 增量式播种：上一帧解 + 编舞增量，保证解连续且跟随编舞
        q_init = q_sol + (ref - prev_ref)
        q_sol, _ = weighted_clik_fixed_tcp(
            model, data, fid, tcp_target, q_init,
            GROOVE["w_body"], GROOVE["max_iter"], GROOVE["tol"])
        frames.append(q_sol)
        prev_ref = ref
    return frames, [tcp_target.copy()] * n_frames


# =============================================================================
# 段2：空间滑步 —— 官方 solve_ik 全位姿 IK + 链式初值
# =============================================================================

def slide_waypoints(home_pose):
    """返回 [(pos(3,), duration), ...]：home→四角→home，目标姿态恒为 home。"""
    s = SLIDE
    x, yh = s["x"], s["y_half"]
    zt, zb = s["z_top"], s["z_bottom"]
    pts = [
        (np.array([x, yh, zt]), s["lead_in"]),    # home -> 左上
        (np.array([x, yh, zb]), s["edge"]),       # 左上 -> 左下
        (np.array([x, -yh, zb]), s["edge"]),      # 左下 -> 右下
        (np.array([x, -yh, zt]), s["edge"]),      # 右下 -> 右上
        (np.array([x, yh, zt]), s["edge"]),       # 右上 -> 左上（闭合成矩形）
        (home_pose.translation.copy(), s["lead_out"]),  # 左上 -> home
    ]
    return pts


def gen_slide(model, data, fid, home_pose):
    wps = slide_waypoints(home_pose)
    frames, targets = [], []
    q6 = HOME_Q.copy()
    q_full = np.zeros(model.nq)
    pos = home_pose.translation.copy()
    params = IKParams(max_iter=SLIDE["max_iter"], tolerance=SLIDE["tol"])
    for goal, duration in wps:
        n = max(2, int(round(duration * RATE_HZ)))
        for k in range(1, n + 1):
            u = k / n
            eased = 0.5 - 0.5 * math.cos(math.pi * u)   # 边角缓动
            tgt_pos = pos + (goal - pos) * eased
            q_full[:6] = q6
            res = solve_ik(model, data, fid, pos_rot_to_se3(tgt_pos), q_full,
                           params, controlled_joints=6, position_only=True)
            if not res.success:
                raise RuntimeError(
                    f"slide IK 未收敛于 {tgt_pos}, err={res.error:.2e}")
            q6 = res.q[:6].copy()
            frames.append(q6)
            targets.append(tgt_pos)
        pos = goal
    return frames, targets


# =============================================================================
# 校验
# =============================================================================

def verify_track(name, frames, targets, model, data, fid, ref_rot=None):
    """FK 误差 / 插值中点漂移 / 限位余量 / 关节速度 四项校验。"""
    arr = np.asarray(frames)
    worst_pos = worst_mid = worst_rot = 0.0
    ref_z = ref_rot[:, 2] if ref_rot is not None else None
    for i, q6 in enumerate(arr):
        err = float(np.linalg.norm(fk_pos(model, data, fid, q6) - targets[i]))
        worst_pos = max(worst_pos, err)
        if ref_z is not None:
            q = np.zeros(model.nq); q[:6] = q6
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            z = data.oMf[fid].rotation[:, 2]
            worst_rot = max(worst_rot, math.degrees(
                math.acos(float(np.clip(z @ ref_z, -1.0, 1.0)))))
        if i + 1 < len(arr):
            q_mid = 0.5 * (q6 + arr[i + 1])
            p_mid = fk_pos(model, data, fid, q_mid)
            t_mid = 0.5 * (targets[i] + targets[i + 1])
            worst_mid = max(worst_mid, float(np.linalg.norm(p_mid - t_mid)))
    # 首尾闭环检查（groove 循环时相邻）
    loop_gap = float(np.linalg.norm(arr[0] - arr[-1]))
    margin = np.minimum(arr - JOINT_LIMITS_LO, JOINT_LIMITS_HI - arr).min()
    max_vel = float(np.abs(np.diff(arr, axis=0)).max() * RATE_HZ) if len(arr) > 1 else 0.0

    print(f"  [{name}] 帧数={len(arr)} 时长={len(arr)/RATE_HZ:.1f}s")
    print(f"    FK 位置误差 max      = {worst_pos*1000:.3f} mm")
    print(f"    插值中点漂移 max     = {worst_mid*1000:.3f} mm")
    print(f"    首尾帧关节差(循环缝) = {loop_gap:.4f} rad")
    print(f"    限位最小余量         = {margin:.3f} rad")
    print(f"    关节速度 max         = {max_vel:.3f} rad/s")
    j1 = arr[:, 0]
    print(f"    j1 摆幅实际保留      = [{j1.min():+.3f}, {j1.max():+.3f}] rad")
    if ref_z is not None:
        print(f"    姿态漂移 max         = {worst_rot:.1f} deg")

    ok = (worst_pos < MAX_POS_ERR and worst_mid < MAX_POS_ERR
          and margin > LIMIT_MARGIN and max_vel < MAX_JOINT_SPEED)
    if not ok:
        raise RuntimeError(f"轨迹 [{name}] 校验未通过，请调整编舞参数后重跑")
    return {"frames": len(arr), "max_pos_err_mm": round(worst_pos * 1000, 4),
            "max_mid_err_mm": round(worst_mid * 1000, 4),
            "min_limit_margin_rad": round(float(margin), 4),
            "max_joint_speed_rad_s": round(max_vel, 3)}


# =============================================================================
# main
# =============================================================================

def main():
    print(f"URDF: {URDF}")
    model, data, fid = load_dm_model()
    home_pose = home_tcp(model, data, fid)
    tcp = home_pose.translation
    print(f"home TCP (FK): [{tcp[0]:.4f}, {tcp[1]:.4f}, {tcp[2]:.4f}] m")

    print("\n=== 段1 机械街舞（TCP 钉死，身体律动）===")
    groove_frames, groove_targets = gen_groove(model, data, fid, tcp.copy())
    groove_report = verify_track("groove", groove_frames, groove_targets,
                                 model, data, fid)

    print("\n=== 段2 空间滑步（矩形轨迹，姿态朝前）===")
    slide_frames, slide_targets = gen_slide(model, data, fid, home_pose)
    slide_report = verify_track("slide", slide_frames, slide_targets,
                                model, data, fid, ref_rot=home_pose.rotation)

    # 结束 pose 体检（从 dance.yaml 读，FK 打印 TCP 位置 + 限位余量）
    if DANCE_CFG.exists():
        cfg = yaml.safe_load(DANCE_CFG.read_text())["dance"]
        fp = cfg.get("final_pose")
        if fp:
            q6 = np.array([fp[f"joint{i}"] for i in range(1, 7)])
            p = fk_pos(model, data, fid, q6)
            margin = np.minimum(q6 - JOINT_LIMITS_LO, JOINT_LIMITS_HI - q6).min()
            print(f"\n=== 结束 pose FK ===")
            print(f"  TCP = [{p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}] m  限位余量 {margin:.3f} rad")

    out = {
        "#": "本文件由 precompute_dance.py 自动生成，请勿手改；改动作请改脚本参数后重跑",
        "rate_hz": RATE_HZ,
        "home_tcp": [round(float(v), 5) for v in tcp],
        "tracks": {
            "groove": {
                "description": "机械街舞一圈（首尾=home，可循环）",
                "report": groove_report,
                "frames": [[round(float(v), 5) for v in q] for q in groove_frames],
            },
            "slide": {
                "description": "空间滑步 home→左上→左下→右下→右上→左上→home",
                "report": slide_report,
                "frames": [[round(float(v), 5) for v in q] for q in slide_frames],
            },
        },
    }
    OUT_YAML.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT_YAML, "w", encoding="utf-8") as f:
        yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False, width=120)
    print(f"\n已写出 {OUT_YAML}")


if __name__ == "__main__":
    main()
