"""DM manual motion failures must replace persistent motor targets with a stop."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import Mock

spec=importlib.util.spec_from_file_location('manual_driver_server',Path(__file__).resolve().parents[1]/'server.py')
server=importlib.util.module_from_spec(spec);spec.loader.exec_module(server)


class ManualDriverTests(unittest.TestCase):
    def bridge(self):
        bridge=server.RosBridge.__new__(server.RosBridge)
        bridge.lock=threading.RLock();bridge.stop_event=threading.Event();bridge.manual_motion={}
        bridge._ensure_original_available=Mock();bridge._check=Mock()
        bridge.fake_publishers=[Mock() for _ in server.JOINTS]
        for publisher in bridge.fake_publishers:publisher.get_subscription_count.return_value=1
        bridge.state=Mock(return_value={'positions':dict.fromkeys(server.JOINTS,0.),'control_mode':'pos_vel'})
        bridge.JointPosVelCmd=lambda:SimpleNamespace()
        bridge.node=Mock();bridge.Trigger=SimpleNamespace(Request=lambda:object())
        bridge.triggers={'safety_stop':Mock()};bridge._await=Mock()
        return bridge

    def test_failed_feedback_stops_the_original_controller_after_one_target(self):
        bridge=self.bridge();bridge._check.side_effect=[None,server.BridgeError('Feedback missing or stale',409)]
        with self.assertRaisesRegex(server.BridgeError,'Feedback missing'):
            bridge._manual_pos_vel({'joint':'joint4','position':.4,'speed':.1})
        publisher=bridge.fake_publishers[3]
        publisher.publish.assert_called_once()
        message=publisher.publish.call_args.args[0]
        self.assertEqual((message.pos,message.vlim),(.4,.1))
        for index,other in enumerate(bridge.fake_publishers):
            if index!=3:other.publish.assert_not_called()
        self.assertTrue(bridge.stop_event.is_set())
        bridge.triggers['safety_stop'].call_async.assert_called_once()

    def test_missing_subscriber_does_not_dispatch_a_motor_target(self):
        bridge=self.bridge();bridge.fake_publishers[0].get_subscription_count.return_value=0
        bridge.stop_event.set()
        with self.assertRaises(server.BridgeError):bridge._manual_pos_vel({'joint':'joint1','position':.4,'speed':.3})
        for publisher in bridge.fake_publishers:publisher.publish.assert_not_called()
        bridge.triggers['safety_stop'].call_async.assert_not_called()


if __name__=='__main__':unittest.main()
