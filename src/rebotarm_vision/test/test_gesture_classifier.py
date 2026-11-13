"""Geometry regressions; synthetic points, not a saved camera recording."""
import math
import unittest

from rebotarm_vision.gesture_classifier import (
    DEFAULT_PARAMS, INDEX_MCP, INDEX_PIP, INDEX_TIP, THUMB_TIP,
    WRIST, MIDDLE_MCP, bend_angle_deg, classify, heart_debug,
)


def heart_hands(curved=False):
    left = [(-1.0, -0.6)] * 21
    left[WRIST] = (-1.8, 0.0)
    left[MIDDLE_MCP] = (-1.0, -0.6)
    left[THUMB_TIP] = (-0.05, -0.4)
    left[INDEX_MCP] = (-1.0, -0.65)
    left[INDEX_PIP] = (-1.05, -1.5) if curved else (-0.55, -1.2)
    left[INDEX_TIP] = (-0.1, -1.75)
    return [left, [(-x, y) for x, y in left]]


class HeartClassificationTest(unittest.TestCase):
    def test_heart_with_projected_straight_indices_is_accepted(self):
        hands = heart_hands()
        for hand in hands:
            self.assertLess(bend_angle_deg(hand, INDEX_MCP, INDEX_PIP, INDEX_TIP), 1)
        self.assertEqual(classify(hands), 'heart')

    def test_curved_heart_is_still_accepted(self):
        self.assertEqual(classify(heart_hands(curved=True)), 'heart')

    def test_rotation_reflection_scale_and_hand_order_do_not_change_result(self):
        for scale in (0.25, 1, 100):
            for angle in (0, 0.7, 2.4):
                for reflection in (1, -1):
                    c, s = math.cos(angle), math.sin(angle)
                    hands = [[(scale * (reflection*x*c-y*s)+400,
                               scale * (reflection*x*s+y*c)+200)
                              for x, y in hand] for hand in heart_hands()]
                    self.assertEqual(classify(hands[::-1]), 'heart')

    def test_close_parallel_open_hands_are_not_heart(self):
        hands = heart_hands()
        for i, hand in enumerate(hands):
            sign = -1 if i == 0 else 1
            hand[INDEX_MCP] = (sign * 0.25, -0.65)
            hand[INDEX_PIP] = (sign * 0.25, -1.2)
            hand[INDEX_TIP] = (sign * 0.25, -1.75)
        rows = heart_debug(*hands)
        self.assertTrue(all(row[3] for row in rows[:5]))
        self.assertFalse(any(row[3] for row in rows[5:]))
        self.assertEqual(classify(hands), 'none')

    def test_fists_and_separated_hands_are_not_heart(self):
        hands = heart_hands()
        hands[1] = [(x+3, y) for x, y in hands[1]]
        self.assertEqual(classify(hands), 'none')
        hands = heart_hands()
        for hand in hands:
            hand[THUMB_TIP] = hand[INDEX_TIP]
        self.assertEqual(classify(hands), 'none')

    def test_one_hand_and_no_hands_never_produce_heart(self):
        self.assertNotEqual(classify(heart_hands()[:1]), 'heart')
        self.assertEqual(classify([]), 'none')

    def test_debug_and_classification_use_identical_thresholds(self):
        params = dict(DEFAULT_PARAMS, heart_index_inward_min=2.0)
        hands = heart_hands()
        self.assertEqual(classify(hands, params), 'none')
        self.assertFalse(all(row[3] for row in heart_debug(*hands, params)))
