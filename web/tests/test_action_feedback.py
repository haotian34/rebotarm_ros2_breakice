"""Action acknowledgements survive missing DDS status without masking stalls."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec=importlib.util.spec_from_file_location('action_worker_test',Path(__file__).resolve().parents[1]/'action_worker.py')
worker=importlib.util.module_from_spec(spec);spec.loader.exec_module(worker)


class FeedbackTests(unittest.TestCase):
    def record(self,file,event):
        file.seek(0,2);file.write((worker.PREFIX+json.dumps(event,ensure_ascii=False)+'\n').encode());file.flush()

    def test_local_original_status_survives_all_missing_ros_status(self):
        feedback=worker.Feedback()
        with tempfile.TemporaryFile() as file:
            self.record(file,{'event':'status','text':'开始表演：舞蹈（机械街舞 + 空间滑步）'})
            feedback.read(file,1)
            self.assertTrue(feedback.began);self.assertEqual(feedback.source,'local_worker')
            for now in range(2,32):
                self.record(file,{'event':'progress','cursor':['RUNNING',now,.1],'step':now,'phase':'RUNNING'})
                feedback.read(file,now)
                self.assertEqual(feedback.timeout(now,0,.5,22),'')
            self.record(file,{'event':'status','text':'表演结束，回到就绪状态'})
            feedback.read(file,32);self.assertTrue(feedback.finished)

    def test_stale_ros_idle_cannot_finish_a_new_running_invocation(self):
        feedback=worker.Feedback();feedback.status('开始表演',10,'local_worker')
        feedback.status('IDLE',8,'ros')
        self.assertFalse(feedback.finished)
        feedback.status('IDLE',11,'ros');self.assertFalse(feedback.finished)
        feedback.status('IDLE',12,'local_worker');self.assertTrue(feedback.finished)

    def test_progress_itself_acknowledges_running_original_tick(self):
        feedback=worker.Feedback()
        with tempfile.TemporaryFile() as file:
            self.record(file,{'event':'progress','cursor':['TRACK',12,.3],'step':2,'phase':'TRACK'})
            feedback.read(file,3)
        self.assertTrue(feedback.began);self.assertEqual(feedback.step,2)
        self.assertEqual(feedback.timeout(10,0,1,20),'')
        self.assertIn('没有推进',feedback.timeout(11.1,0,1,20))

    def test_heartbeat_with_unchanged_cursor_does_not_hide_stalled_execution(self):
        feedback=worker.Feedback()
        with tempfile.TemporaryFile() as file:
            for now in (1,2,3,8,10):
                self.record(file,{'event':'progress','cursor':['RUNNING',1,.2],'step':2,'phase':'RUNNING'})
                feedback.read(file,now)
        self.assertIn('没有推进',feedback.timeout(10,0,.5,20))

    def test_partial_utf8_log_record_and_unrelated_node_output(self):
        feedback=worker.Feedback()
        payload=(worker.PREFIX+json.dumps({'event':'status','text':'开始表演'},ensure_ascii=False)+'\n').encode()
        with tempfile.TemporaryFile() as file:
            file.write(b'ordinary ROS log\n'+payload[:-4]);file.flush();feedback.read(file,1)
            self.assertFalse(feedback.began)
            file.seek(0,2);file.write(payload[-4:]);file.flush();feedback.read(file,2)
        self.assertTrue(feedback.began)

    def test_untriggered_and_nonresponsive_nodes_still_time_out(self):
        feedback=worker.Feedback()
        self.assertIn('未确认',feedback.timeout(13,0,None,20))
        self.assertEqual(feedback.timeout(20,0,15,20),'')


if __name__=='__main__':unittest.main()
