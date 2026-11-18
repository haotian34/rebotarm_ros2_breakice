"""Observe original action execution over an owned local process log.

ROS status topics remain unchanged. A local copy of each original status and
tick cursor prevents one-shot DDS status discovery from hiding a running action.
No targets, timing, choreography or hardware code are replaced.
"""
import importlib
import json
import os
import sys
import time

PREFIX='REBOTARM_ACTION_EVENT '
NODES={'greeting':('rebotarm_greeting.greeting_node','GreetingNode'),
       'dance':('rebotarm_dance.dance_node','DanceNode'),
       'action':('rebotarm_gesture_actions.trajectory_player_node','TrajectoryPlayerNode')}


def observe_node(cls,emit):
    status,tick=cls._publish_status,cls._tick
    def publish_status(self,text):
        status(self,text)
        emit({'event':'status','text':text})
    def observed_tick(self):
        tick(self)
        now=time.monotonic()
        if now-getattr(self,'_web_observed_at',0)<.5:return
        self._web_observed_at=now
        state=self.state
        if state=='IDLE':return
        if hasattr(self,'step_index'):
            cursor=[state,self.step_index,self.step_t]
            step=self.step_index+1
        else:
            cursor=[state,self.frame_index,self.entry_t,getattr(self,'reset_t',0)]
            step={'ENTRY':1,'TRACK':2,'RESET':3}.get(state,1)
        emit({'event':'progress','cursor':cursor,'step':step,'phase':state})
    cls._publish_status,cls._tick=publish_status,observed_tick


class Feedback:
    def __init__(self):
        self.offset=0
        self.pending=b''
        self.began=self.finished=False
        self.started=self.last_progress=0.
        self.cursor=None
        self.step=None
        self.phase=''
        self.text=self.source=''
        self.local_seen=False

    def status(self,text,now,source):
        if source=='local_worker':self.local_seen=True
        elif self.local_seen:return  # Delayed DDS IDLE must not end a local run.
        if self.began and now<self.started:return
        self.text,self.source=text,source
        if text.startswith('BUSY:') or '开始表演' in text:
            if not self.began:
                self.began=True;self.started=self.last_progress=now
        elif self.began and (text=='IDLE' or '表演结束' in text):
            self.finished=True

    def read(self,file,now):
        data=os.pread(file.fileno(),65536,self.offset)
        self.offset+=len(data)
        self.pending+=data
        lines=self.pending.split(b'\n');self.pending=lines.pop()[-65536:]
        for line in lines:
            if not line.startswith(PREFIX.encode()):continue
            try:event=json.loads(line[len(PREFIX):])
            except (ValueError,UnicodeError):continue
            if event.get('event')=='status':
                self.status(event['text'],now,'local_worker')
            elif event.get('event')=='progress':
                self.local_seen=True;self.source='local_worker'
                if not self.began:self.began=True;self.started=now
                if event['cursor']!=self.cursor:
                    self.last_progress=now;self.cursor=event['cursor']
                self.step,self.phase=event['step'],event['phase']

    def timeout(self,now,invoked,triggered_at,duration):
        if not self.began:
            if now-(triggered_at or invoked)>12:
                return '原项目动作节点未确认开始执行，请检查动作节点日志。'
        elif not self.finished:
            if now-self.last_progress>8:
                return '原项目动作步骤持续 8 秒没有推进，已停止并锁止。'
            if now-self.started>max(30,duration*3+15):
                return '原项目动作执行超过最长等待时间，已停止并锁止。'
        return ''


def main():
    if len(sys.argv)<2 or sys.argv[1] not in NODES:raise SystemExit('Unsupported action worker')
    module_name,class_name=NODES[sys.argv[1]]
    module=importlib.import_module(module_name)
    def emit(event):print(PREFIX+json.dumps(event,ensure_ascii=False,separators=(',',':')),flush=True)
    observe_node(getattr(module,class_name),emit)
    module.main(args=sys.argv[2:])


if __name__=='__main__':main()
