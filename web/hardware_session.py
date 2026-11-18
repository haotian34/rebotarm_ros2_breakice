"""Owned DM hardware connection. GET diagnostics never opens the serial bus."""
import grp
import os
from pathlib import Path
import pwd
import re
import shlex
import shutil
import signal
import subprocess
import tempfile
import threading
import time

def serial_devices(dev=Path('/dev')):
    candidates={}
    for path in sorted((dev/'serial/by-id').glob('*'))+sorted(dev.glob('ttyACM*'))+sorted(dev.glob('ttyUSB*')):
        try:
            resolved=path.resolve(strict=True)
            if resolved.parent!=dev or not re.fullmatch(r'tty(?:ACM|USB)\d+',resolved.name) or not resolved.is_char_device():
                continue
            candidates.setdefault(str(resolved),path)
        except OSError:
            continue
    result=[]
    for resolved,path in candidates.items():
        try:
            info=path.stat()
            group=grp.getgrgid(info.st_gid)
        except (OSError,KeyError):
            continue
        refresh=not os.access(path,os.R_OK|os.W_OK) and group.gr_gid!=0 and pwd.getpwuid(os.getuid()).pw_name in group.gr_mem and bool(shutil.which('sg'))
        owners=[]
        for directory in Path('/proc').glob('[0-9]*/fd'):
            try:
                if any(os.readlink(fd)==resolved for fd in directory.iterdir()):
                    owners.append(int(directory.parent.name))
            except (OSError,PermissionError):
                continue
        result.append({'path':str(path),'device':resolved,'label':path.name,
                       'accessible':bool(os.access(path,os.R_OK|os.W_OK) or refresh),
                       'refresh_group':group.gr_name if refresh else None,'owners':owners})
    return result


class HardwareSession:
    def __init__(self,bridge_factory,demo_factory,error_type,domain_id=188,namespace='rebotarm_web_hardware'):
        self.factory,self.demo,self.Error=bridge_factory,demo_factory,error_type
        self.domain_id,self.namespace=domain_id,namespace
        self.lock=threading.RLock()
        self.status,self.error,self.device='stopped','',''
        self.stage='idle'
        self.bridge=None
        self.process=None
        self.worker=None
        self.stop_event=threading.Event()
        self.identity=None
        self.joint_count=0

    def runtime(self):
        with self.lock:
            return {'managed':True,'kind':'hardware','model':'dm','namespace':self.namespace,'domain_id':self.domain_id,
                    'status':self.status,'error':self.error,'device':self.device,'stage':self.stage,'joint_count':self.joint_count}

    def state(self):
        with self.lock:
            bridge=self.bridge if self.status=='running' else None
        state=bridge.state() if bridge else self.demo('dm',self.namespace).state()
        return {**state,'runtime':self.runtime()}

    def select(self,device=''):
        devices=serial_devices()
        if device:
            match=next((item for item in devices if device in (item['path'],item['device'])),None)
            if match is None:
                raise self.Error('串口不存在或已拔出，请刷新设备列表。',409)
        elif len(devices)==1:
            match=devices[0]
        else:
            raise self.Error('未检测到 USB 串口，请连接机械臂并刷新。' if not devices else '检测到多个串口，请先选择机械臂使用的设备。',409)
        if not match['accessible']:
            raise self.Error('当前账户没有串口读写权限，请先在系统中授予该设备权限；网页不会修改系统权限。',403)
        if match['owners']:
            raise self.Error('串口正被其他程序占用，请先关闭占用该串口的控制程序。',409)
        return match

    def start(self,selection):
        with self.lock:
            if self.status in ('starting','running','stopping'):
                raise self.Error('真机正在连接或已连接。',409)
            self.device=selection['path']
            info=Path(self.device).stat()
            self.identity=(info.st_dev,info.st_ino,info.st_rdev)
            self.status,self.stage,self.error,self.joint_count='starting','driver','',0
            self.stop_event.clear()
            self.worker=threading.Thread(target=self._run,args=(selection,),name='web-dm-hardware',daemon=True)
            self.worker.start()
        return self.runtime()

    def _device_present(self):
        try:
            value=Path(self.device).stat()
            return (value.st_dev,value.st_ino,value.st_rdev)==self.identity
        except OSError:
            return False

    def _run(self,selection):
        bridge=process=None
        failure=''
        with tempfile.TemporaryFile(mode='w+') as output:
            try:
                if not self._device_present():
                    raise RuntimeError('串口已拔出，请重新选择设备。')
                bridge=self.factory('dm',self.namespace,True,domain_id=self.domain_id)
                command=['/opt/ros/'+os.environ.get('ROS_DISTRO','jazzy')+'/bin/ros2',
                         'launch','rebotarm_bringup','driver.launch.py',
                         'model:=dm',f'channel:={self.device}',f'arm_namespace:={self.namespace}',
                         'joint_state_rate:=100.0']
                if selection.get('refresh_group'):
                    command=['sg',selection['refresh_group'],'-c',shlex.join(command)]
                env=os.environ.copy()
                env.update(ROS_DOMAIN_ID=str(self.domain_id),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_LOCALHOST_ONLY='1',PYTHONUNBUFFERED='1')
                process=subprocess.Popen(command,env=env,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
                with self.lock:
                    self.bridge,self.process,self.stage=bridge,process,'feedback'
                deadline=time.monotonic()+25
                while not self.stop_event.wait(.1):
                    if process.poll() is not None:
                        detail=os.pread(output.fileno(),65536,0).decode(errors='replace')[-1400:]
                        raise RuntimeError('原 DM 驱动未能保持连接：'+detail)
                    if not self._device_present():
                        raise RuntimeError('USB 串口已断开，控制已停止；重新插入后请手动连接。')
                    state=bridge.state()
                    self.joint_count=sum(name in state['positions'] for name in (f'joint{i}' for i in range(1,7)))
                    fresh=state['joint_age_sec'] is not None and state['joint_age_sec']<1.5 and state['status_age_sec'] is not None and state['status_age_sec']<2
                    if self.status=='starting':
                        if fresh and self.joint_count==6 and state.get('control_mode') in ('pos_vel','mit'):
                            with self.lock:
                                self.status,self.stage='running','ready'
                        elif time.monotonic()>deadline:
                            raise RuntimeError('连接超时：未收到原 DM 驱动的六轴关节与控制器反馈，请检查供电、线缆和设备选择。')
                    elif not fresh:
                        raise RuntimeError('真机反馈中断，控制已停止；请检查连接后手动重连。')
            except Exception as exc:
                if not self.stop_event.is_set():failure=str(exc)
            finally:
                stop_confirmed=False
                if bridge:
                    bridge.allow_motion=False
                    bridge.cancel_manual()
                    # Independent safety stop remains allowed on stale feedback.
                    if process and process.poll() is None:
                        try:
                            result=bridge.command({'command':'safety_stop'})
                            stop_confirmed=bool(result.get('success'))
                        except Exception:pass
                        if not stop_confirmed:
                            try:stop_confirmed=bridge.state().get('state_machine') in ('SAFETY_STOPPED','SAFETY_FAULT')
                            except Exception:pass
                if process and process.poll() is None:
                    try:
                        # Original graceful shutdown homes unless safety-latched.
                        # Never enter that fallback when the stop was unconfirmed.
                        if not stop_confirmed:
                            failure=failure or '停止反馈未确认，已结束本页驱动进程；请检查机械臂实际状态后再连接。'
                        os.killpg(process.pid,signal.SIGINT if stop_confirmed else signal.SIGKILL)
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid,signal.SIGKILL)
                        process.wait(timeout=3)
                        failure=failure or '驱动退出超时，连接已关闭，请检查机械臂状态。'
                    except ProcessLookupError:pass
                if process and process.returncode not in (None,0,130,-signal.SIGINT):
                    failure=failure or '驱动已退出，但停止或失能未完全确认，请检查机械臂状态。'
                if bridge:
                    try:bridge.close()
                    except Exception as exc:failure=failure or str(exc)
                with self.lock:
                    self.bridge=self.process=None
                    self.status='error' if failure else 'stopped'
                    self.stage='error' if failure else 'idle'
                    self.error=failure

    def stop(self):
        with self.lock:
            if self.status in ('starting','running'):
                self.status='stopping';self.stage='stopping';self.stop_event.set()
                if self.bridge:
                    self.bridge.allow_motion=False
                    self.bridge.cancel_manual()
        return self.runtime()

    def close(self):
        self.stop()
        if self.worker:self.worker.join(timeout=22)
        if self.worker and self.worker.is_alive():
            raise RuntimeError('真机驱动仍在退出，暂不能切换到仿真。')
