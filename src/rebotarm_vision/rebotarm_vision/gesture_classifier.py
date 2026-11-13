"""
手势几何分类器（纯函数，不依赖 ROS，可被测试脚本和 ROS 节点共用）

输入：MediaPipe Hands 的 21 关键点（像素坐标），输出标签：
  rock   ✊ 石头：食/中/无名/小 4 指全弯（拇指不参与，个体差异大）
  paper  ✋ 布：  4 指全伸
  heart  🫶 双手比心：两只手同时入镜，两手拇指尖相碰、食指尖相碰
  none   无 / 不确定

判定规则：双手入镜只判 heart / none；单手入镜才判 rock / paper / none

距离度量说明：
  单手弯曲度 = dist(指尖, 手腕) / dist(指根MCP, 手腕)，
    弯曲时指尖收向掌心，比值 < ~1.1；伸直时比值 > ~1.6。
  双手比心的距离阈值按「手尺度」= dist(手腕, 中指根MCP) 归一化，
    与手离摄像头的远近无关。
"""

import math

# MediaPipe Hands 21 关键点索引
WRIST = 0
THUMB_TIP = 4
INDEX_MCP, INDEX_PIP, INDEX_TIP = 5, 6, 8
MIDDLE_MCP, MIDDLE_TIP = 9, 12
RING_MCP, RING_TIP = 13, 16
PINKY_MCP, PINKY_TIP = 17, 20

# 参与石头/布判断的 4 根手指（指根, 指尖），拇指不参与
FINGERS = [
    ("index", INDEX_MCP, INDEX_TIP),
    ("middle", MIDDLE_MCP, MIDDLE_TIP),
    ("ring", RING_MCP, RING_TIP),
    ("pinky", PINKY_MCP, PINKY_TIP),
]

# 默认阈值（可用各字段覆盖；先用 scripts/test_gesture.py 实测调节）
DEFAULT_PARAMS = {
    "rock_ratio_max": 1.15,   # 4 指比值全部 < 此值 → rock
    "paper_ratio_min": 1.50,  # 4 指比值全部 > 此值 → paper
    "heart_tip_max": 0.8,     # 两手拇指尖间距 < 0.8 * 手尺度
    "heart_index_max": 0.8,   # 两手食指尖间距 < 0.8 * 手尺度
    "heart_wrist_min": 1.2,   # 两手腕间距 > 1.2 * 手尺度（确为分开的两只手）
    "heart_spread_min": 0.5,  # 同手拇指尖-食指尖间距 > 0.5 * 手尺度（比心是张开的）
    # 食指尖沿两手食指根的连线向内靠拢。屏幕投影中的食指可以是直的，
    # 不再要求 PIP 弯曲角；仍排除两只平行张开的手掌。
    "heart_index_inward_min": 0.15,
}


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def hand_scale(lm):
    """手尺度：手腕到中指根的距离（像素），用于阈值归一化。"""
    return _dist(lm[WRIST], lm[MIDDLE_MCP]) + 1e-6


def bend_angle_deg(lm, a, b, c):
    """关节屈曲角：顶点 b（PIP），向量 b->a（MCP）与 b->c（TIP）的夹角。
    完全伸直时两向量反向共线 -> 夹角 180°；这里返回 180° - 夹角，
    即伸直=0°，屈曲越大角度越大。"""
    v1 = (lm[a][0] - lm[b][0], lm[a][1] - lm[b][1])
    v2 = (lm[c][0] - lm[b][0], lm[c][1] - lm[b][1])
    n1 = math.hypot(*v1) + 1e-6
    n2 = math.hypot(*v2) + 1e-6
    cos = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
    return 180.0 - math.degrees(math.acos(cos))


def finger_ratios(lm):
    """返回 {指名: dist(指尖,腕)/dist(指根,腕)}，供分类与调试显示。"""
    out = {}
    for name, mcp, tip in FINGERS:
        out[name] = _dist(lm[tip], lm[WRIST]) / (_dist(lm[mcp], lm[WRIST]) + 1e-6)
    return out


def classify_single(lm, p=None):
    """单手分类：返回 'rock' / 'paper' / 'none'。"""
    p = p or DEFAULT_PARAMS
    r = finger_ratios(lm)
    vals = list(r.values())
    if max(vals) < p["rock_ratio_max"]:
        return "rock"
    if min(vals) > p["paper_ratio_min"]:
        return "paper"
    return "none"


def is_heart(lm0, lm1, p=None):
    """Use the same geometric checks as the debug overlay."""
    return all(row[3] for row in heart_debug(lm0, lm1, p))


def heart_debug(lm0, lm1, p=None):
    """比心逐项诊断，返回 [(条件名, 实测值, 阈值, 是否通过), ...]。
    供 scripts/test_gesture.py 实时显示，定位是哪一条不满足。"""
    p = p or DEFAULT_PARAMS
    s = 0.5 * (hand_scale(lm0) + hand_scale(lm1))
    d_wrist = _dist(lm0[WRIST], lm1[WRIST]) / s
    d_thumb = _dist(lm0[THUMB_TIP], lm1[THUMB_TIP]) / s
    d_index = _dist(lm0[INDEX_TIP], lm1[INDEX_TIP]) / s
    rows = [
        ("wrists apart(>)", d_wrist, p["heart_wrist_min"],
         d_wrist >= p["heart_wrist_min"]),
        ("thumb tips(<)", d_thumb, p["heart_tip_max"],
         d_thumb <= p["heart_tip_max"]),
        ("index tips(<)", d_index, p["heart_index_max"],
         d_index <= p["heart_index_max"]),
    ]
    for i, lm in enumerate((lm0, lm1)):
        spread = _dist(lm[THUMB_TIP], lm[INDEX_TIP]) / s
        rows.append((f"hand{i} spread(>)", spread, p["heart_spread_min"],
                     spread >= p["heart_spread_min"]))
    for i, (lm, other) in enumerate(((lm0, lm1), (lm1, lm0))):
        axis = (other[INDEX_MCP][0] - lm[INDEX_MCP][0],
                other[INDEX_MCP][1] - lm[INDEX_MCP][1])
        tip = (lm[INDEX_TIP][0] - lm[INDEX_MCP][0],
               lm[INDEX_TIP][1] - lm[INDEX_MCP][1])
        inward = (tip[0] * axis[0] + tip[1] * axis[1]) / (math.hypot(*axis) + 1e-6) / s
        rows.append((f"hand{i} index inward(>)", inward, p["heart_index_inward_min"],
                     inward >= p["heart_index_inward_min"]))
    return rows


def classify(hands, p=None):
    """总入口。hands: 21x2 像素坐标列表的列表（0~2 只手）。返回标签字符串。

    双手入镜时只判 heart / none（不判石头/布，避免双手布等歧义场景）；
    单手入镜才判 rock / paper / none。
    """
    p = p or DEFAULT_PARAMS
    if len(hands) >= 2:
        return "heart" if is_heart(hands[0], hands[1], p) else "none"
    if len(hands) == 1:
        return classify_single(hands[0], p)
    return "none"
