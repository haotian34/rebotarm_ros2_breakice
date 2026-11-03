#!/usr/bin/env python3
"""
reBot Arm 任务三：手势动作轨迹离线预计算

用官方 SDK（Pinocchio 位置-only IK，链式初值）把三条 TCP 轨迹离线解算成
30Hz 关节轨迹，写入 config/gesture_tracks.yaml；运行时
trajectory_player_node 只做逐帧回放，不在线解 IK。

三条轨迹均以「悬空鸡脖子」home 姿态为首尾锚点（TCP = 前方0.284m/高0.459m）：

  rock_push    ✊ 出拳：TCP 沿基座 X 轴前后平移，±6cm，快出慢收 ×3
  paper_circle ✋ 画圆（旧版小动作，保留在轨迹库中）
  paper_figure8 ✋ 大8字：TCP 在 Y-Z 竖直面画大号双纽线（26x14cm）×2，
               全身关节参与 + j6 手腕扭转
  heart_draw   🫶 两笔画心：右半心(顶部->底部) -> 抬笔回顶部 -> 左半心(顶部->底部)，
               爱心参数曲线 x=16sin³t, z=13cost−5cos2t−2cos3t−cos4t，高 22cm

注意：画圆/爱心的展示平面是 Y-Z（左右×上下），正对机械臂前方的观众
看到的就是完整的圆/爱心；X-Z 面从正面看会退化成竖线。

每帧校验（任一不达标即报错退出、不写文件，与 precompute_dance 一致）：
  1. FK 位置误差 < 2mm
  2. 相邻帧线性插值中点的 FK 漂移 < 2mm
  3. 关节限位余量 > 0.05rad，相邻帧关节速度 < 0.95 rad/s

用法：
  python3 src/rebotarm_gesture_actions/rebotarm_gesture_actions/precompute_gesture_tracks.py
"""

import math
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml

# ---- 定位工作空间与官方 SDK ------------------------------------------------
# 本文件: <ws>/src/rebotarm_gesture_actions/rebotarm_gesture_actions/precompute_gesture_tracks.py
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
OUT_YAML = Path(__file__).resolve().parents[1] / "config" / "gesture_tracks.yaml"

# ---- 全局参数 ---------------------------------------------------------------
RATE_HZ = 30.0                 # 轨迹采样率（与播放器节点节拍一致）
MAX_POS_ERR = 0.002            # TCP 位置误差上限 2mm
LIMIT_MARGIN = 0.05            # 关节限位最小余量（rad）
MAX_JOINT_SPEED = 0.95         # 相邻帧关节速度上限（驱动限 1.0，留余量）

HOME_Q = np.array([0.0, -0.62, -0.85, 0.0, 0.0, 0.0])   # 悬空鸡脖子
JOINT_LIMITS_LO = np.array([-2.8, -3.14, -3.14, -1.87, -1.57, -3.14])
JOINT_LIMITS_HI = np.array([2.8, 0.0, 0.0, 1.57, 1.57, 3.14])

# ---- 编舞参数 ---------------------------------------------------------------
ROCK_PUSH = {
    "amplitude": 0.06,   # 出拳幅度 ±6cm（沿基座 X 轴）
    "retract_in": 0.8,   # home -> 后拉点
    "punch": 0.85,       # 后拉点 -> 前伸点（快出，像出拳）
    "retract": 1.0,     # 前伸点 -> 后拉点（慢收）
    "times": 3,
    "return_home": 0.6,  # 后拉点 -> home
    "max_iter": 2000,
    "tol": 1e-7,
}

PAPER_CIRCLE = {
    "radius": 0.05,      # 直径 10cm
    "laps": 2,
    "lap_period": 2.5,   # 每圈时长
    "lead": 1.0,         # home -> 圆上起点 / 起点 -> home
    "max_iter": 2000,
    "tol": 1e-7,
}

PAPER_FIGURE8 = {
    # 大号 8 字（Gerono 双纽线）：y = A*sin(t), z = z0 + B*sin(t)*cos(t)
    # 全身参与：j1 随横向大幅摆动，j2/j3 随高度起伏，j4/j5 腕部补偿，
    # 另叠加 j6 手腕缓慢扭转（不影响 TCP 位置），展示张开的手掌
    "amp_y": 0.13,       # 横向半幅 13cm（总宽 26cm）
    "amp_z": 0.14,       # 竖直参数（8 字总高 = amp_z = 14cm）
    "period": 4.0,       # 一个 8 字时长
    "laps": 2,
    "wrist_amp": 0.5,    # j6 扭转幅值（rad）
    "max_iter": 2000,
    "tol": 1e-7,
}

HEART_DRAW = {
    "height": 0.22,      # 爱心高度 22cm（宽度随曲线纵横比，约 24.5cm）
    "half_period": 2.5,  # 每个半心时长（右半、左半各画一遍，都从上到下）
    "return_top": 1.2,   # 底部心尖 -> 顶部凹陷 的抬笔回程
    "lead": 1.2,         # home -> 顶部凹陷 / 底部心尖 -> home
    "max_iter": 2000,
    "tol": 1e-7,
}


# =============================================================================
# 模型与基础工具
# =============================================================================

def load_dm_model():
    model = pin.buildModelFromUrdf(str(URDF))
    data = model.createData()
    fid = model.getFrameId("end_link")
    return model, data, fid


def home_tcp(model, data, fid):
    q = np.zeros(model.nq)
    q[:6] = HOME_Q
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    return data.oMf[fid].translation.copy()


def fk_pos(model, data, fid, q6):
    q = np.zeros(model.nq)
    q[:6] = q6
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    return data.oMf[fid].translation.copy()


def ease(u):
    """余弦缓动：两头慢、中间快，u ∈ [0,1]。"""
    return 0.5 - 0.5 * math.cos(math.pi * u)


def solve_path(model, data, fid, positions, max_iter, tol):
    """对一串 TCP 目标点做位置-only IK（链式初值保证解连续）。"""
    frames = []
    q6 = HOME_Q.copy()
    q_full = np.zeros(model.nq)
    params = IKParams(max_iter=max_iter, tolerance=tol)
    for tgt in positions:
        q_full[:6] = q6
        res = solve_ik(model, data, fid, pos_rot_to_se3(np.asarray(tgt)),
                       q_full, params, controlled_joints=6, position_only=True)
        if not res.success:
            raise RuntimeError(f"IK 未收敛于 {tgt}, err={res.error:.2e}")
        q6 = res.q[:6].copy()
        frames.append(q6)
    return frames


def seg_ease(p0, p1, duration):
    """余弦缓动直线段（含终点、不含起点，便于拼接）。"""
    n = max(2, int(round(duration * RATE_HZ)))
    return [p0 + (p1 - p0) * ease((k + 1) / n) for k in range(n)]


# =============================================================================
# 三条轨迹的目标点序列（单位：米，基座坐标系）
# =============================================================================

def rock_push_targets(tcp):
    p = ROCK_PUSH
    x0, y0, z0 = tcp
    back = np.array([x0 - p["amplitude"], y0, z0])
    front = np.array([x0 + p["amplitude"], y0, z0])
    home = tcp.copy()
    pts = seg_ease(home, back, p["retract_in"])
    for _ in range(p["times"]):
        pts += seg_ease(back, front, p["punch"])    # 快出
        pts += seg_ease(front, back, p["retract"])  # 慢收
    pts += seg_ease(back, home, p["return_home"])
    return pts


def paper_circle_targets(tcp):
    p = PAPER_CIRCLE
    x0, y0, z0 = tcp
    r = p["radius"]
    # Y-Z 竖直面画圆：θ=0 在圆顶 (y=0, z=z0+r)
    start = np.array([x0, y0, z0 + r])
    pts = seg_ease(tcp.copy(), start, p["lead"])
    n = max(2, int(round(p["laps"] * p["lap_period"] * RATE_HZ)))
    for k in range(1, n + 1):
        u = k / n                      # 全程余弦缓动，首尾速度为 0
        th = 2.0 * math.pi * p["laps"] * ease(u)
        pts.append(np.array([x0, y0 + r * math.sin(th), z0 + r * math.cos(th)]))
    pts += seg_ease(start, tcp.copy(), p["lead"])
    return pts


def paper_figure8_targets(tcp):
    """大号 8 字：y = A*sin(t)，z = z0 + B*sin(t)*cos(t)，起点即 home 中心。
    返回 (目标点序列, 每帧 j6 扭转角列表)。"""
    p = PAPER_FIGURE8
    x0, y0, z0 = tcp
    a, b = p["amp_y"], p["amp_z"]
    pts = []
    j6 = []
    n = max(2, int(round(p["laps"] * p["period"] * RATE_HZ)))
    for k in range(1, n + 1):
        u = k / n
        t = 2.0 * math.pi * p["laps"] * ease(u)   # 全程余弦缓动，首尾速度为 0
        pts.append(np.array([x0, y0 + a * math.sin(t),
                             z0 + b * math.sin(t) * math.cos(t)]))
        j6.append(p["wrist_amp"] * math.sin(2.0 * math.pi * u))
    return pts, j6


def heart_curve(t):
    """爱心参数曲线（未缩放）：水平 xh，竖直 zh。t ∈ [0, 2π]，首尾闭合。"""
    xh = 16.0 * math.sin(t) ** 3
    zh = (13.0 * math.cos(t) - 5.0 * math.cos(2.0 * t)
          - 2.0 * math.cos(3.0 * t) - math.cos(4.0 * t))
    return xh, zh


def heart_draw_targets(tcp):
    p = HEART_DRAW
    x0, y0, z0 = tcp
    # 先采样完整曲线确定包围盒，再归一化缩放到目标高度并竖直居中
    raw = [heart_curve(2.0 * math.pi * i / 360) for i in range(361)]
    xs = [c[0] for c in raw]
    zs = [c[1] for c in raw]
    s = p["height"] / (max(zs) - min(zs))
    zc = 0.5 * (max(zs) + min(zs))

    def point(t):
        xh, zh = heart_curve(t)
        return np.array([x0, y0 + s * xh, z0 + s * (zh - zc)])

    # 两笔画心：t=0 为顶部凹陷中点，t=π 为底部心尖。
    # 右半（t: 0→π，xh>=0）-> 抬笔回顶部 -> 左半（t: 2π→π，xh<=0），
    # 两个半心都从上到下画出。
    top = point(0.0)
    bottom = point(math.pi)
    n_half = max(2, int(round(p["half_period"] * RATE_HZ)))

    pts = seg_ease(tcp.copy(), top, p["lead"])              # 进场到顶部
    for k in range(1, n_half + 1):                          # 第一笔：右半心
        t = math.pi * ease(k / n_half)
        pts.append(point(t))
    pts += seg_ease(bottom, top, p["return_top"])           # 抬笔回顶部
    for k in range(1, n_half + 1):                          # 第二笔：左半心
        t = 2.0 * math.pi - math.pi * ease(k / n_half)
        pts.append(point(t))
    pts += seg_ease(bottom, tcp.copy(), p["lead"])          # 心尖回 home
    return pts


# =============================================================================
# 校验（与 precompute_dance 相同的四项）
# =============================================================================

def verify_track(name, frames, targets, model, data, fid):
    arr = np.asarray(frames)
    worst_pos = worst_mid = 0.0
    for i, q6 in enumerate(arr):
        err = float(np.linalg.norm(fk_pos(model, data, fid, q6) - targets[i]))
        worst_pos = max(worst_pos, err)
        if i + 1 < len(arr):
            q_mid = 0.5 * (q6 + arr[i + 1])
            p_mid = fk_pos(model, data, fid, q_mid)
            t_mid = 0.5 * (targets[i] + targets[i + 1])
            worst_mid = max(worst_mid, float(np.linalg.norm(p_mid - t_mid)))
    loop_gap = float(np.linalg.norm(arr[0] - arr[-1]))
    margin = np.minimum(arr - JOINT_LIMITS_LO, JOINT_LIMITS_HI - arr).min()
    max_vel = float(np.abs(np.diff(arr, axis=0)).max() * RATE_HZ)

    print(f"  [{name}] 帧数={len(arr)} 时长={len(arr)/RATE_HZ:.1f}s")
    print(f"    FK 位置误差 max      = {worst_pos*1000:.3f} mm")
    print(f"    插值中点漂移 max     = {worst_mid*1000:.3f} mm")
    print(f"    首尾帧关节差         = {loop_gap:.4f} rad")
    print(f"    限位最小余量         = {margin:.3f} rad")
    print(f"    关节速度 max         = {max_vel:.3f} rad/s")

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
    tcp = home_tcp(model, data, fid)
    print(f"home TCP (FK): [{tcp[0]:.4f}, {tcp[1]:.4f}, {tcp[2]:.4f}] m")

    specs = [
        ("rock_push", "✊ 出拳：TCP 沿基座X轴前后 ±6cm，快出慢收×3，首尾=home",
         rock_push_targets, ROCK_PUSH),
        ("paper_circle", "✋ 画圆：TCP 在 Y-Z 竖直面画直径10cm圆×2圈，首尾=home",
         paper_circle_targets, PAPER_CIRCLE),
        ("paper_figure8", "✋ 大8字：TCP 在 Y-Z 竖直面画 26x14cm 双纽线×2圈 + j6 手腕扭转，首尾=home",
         "figure8", PAPER_FIGURE8),
        ("heart_draw", "🫶 两笔画心：右半心(顶->底) -> 抬笔回顶 -> 左半心(顶->底)，高22cm",
         heart_draw_targets, HEART_DRAW),
    ]

    tracks = {}
    for name, desc, gen, cfg in specs:
        print(f"\n=== {name} ===")
        j6_wave = None
        if gen == "figure8":
            targets, j6_wave = paper_figure8_targets(tcp)
        else:
            targets = gen(tcp)
        frames = solve_path(model, data, fid, targets,
                            cfg["max_iter"], cfg["tol"])
        if j6_wave is not None:
            # 叠加 j6 手腕扭转（j6 绕自身轴旋转，不改变 end_link 位置，
            # FK 校验会确认这一点）；首尾归零
            frames = [np.array([*q[:5], w]) for q, w in zip(frames, j6_wave)]
            frames[0] = np.array([*frames[0][:5], 0.0])
            frames[-1] = np.array([*frames[-1][:5], 0.0])
        report = verify_track(name, frames, targets, model, data, fid)
        tracks[name] = {
            "description": desc,
            "report": report,
            "frames": [[round(float(v), 5) for v in q] for q in frames],
        }

    out = {
        "#": "本文件由 precompute_gesture_tracks.py 自动生成，请勿手改；"
             "改动作请改脚本参数后重跑",
        "rate_hz": RATE_HZ,
        "home_tcp": [round(float(v), 5) for v in tcp],
        "tracks": tracks,
    }
    OUT_YAML.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT_YAML, "w", encoding="utf-8") as f:
        yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False, width=120)
    print(f"\n已写出 {OUT_YAML}")


if __name__ == "__main__":
    main()
