"""Run a real RViz window on an isolated authenticated X display, via noVNC."""
from __future__ import annotations
import ctypes
import importlib.util
import os
from pathlib import Path
import secrets
import shutil
import socket
import subprocess
import tempfile
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / 'web/.rviz-runtime'


def available_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def binary(name):
    bundled = RUNTIME / 'usr/bin' / name
    return str(bundled) if bundled.is_file() else shutil.which(name)


class RvizSession:
    def __init__(self, bridge, origins):
        self.bridge = bridge
        self.origins = tuple(origins)
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.worker = None
        self.processes = []
        self.status, self.error = 'stopped', ''
        self.ws_port = None
        self.password = None
        self.generation = 0

    def state(self, credentials=False):
        with self.lock:
            result = {'status': self.status, 'error': self.error,
                      'available': bool(binary('Xvfb') and binary('x11vnc')),
                      'generation': self.generation, 'renderer': 'RViz 2',
                      'width': 1280, 'height': 800}
            if credentials and self.status == 'running':
                result.update(ws_port=self.ws_port, password=self.password)
            return result

    def start(self):
        state = self.bridge.state()
        if state.get('mode') != 'ros':
            raise ValueError('请先启动本机 ROS 2 仿真，或连接真机 ROS 驱动。')
        if state.get('model') != 'dm':
            raise ValueError('当前工作台仅支持 B601 DM。')
        with self.lock:
            if self.status in ('starting', 'running', 'stopping'):
                return self.state()
            if not self.state()['available']:
                raise ValueError('RViz 网页依赖尚未安装，请运行 web/setup_rviz.py。')
            self.stop_event.clear()
            self.status, self.error = 'starting', ''
            self.generation += 1
            self.worker = threading.Thread(target=self._run, args=(state,), name='rviz-web-session', daemon=True)
            self.worker.start()
            return self.state()

    def _spawn(self, args, env, logfile):
        process = subprocess.Popen(args, env=env, stdin=subprocess.DEVNULL,
                                   stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append(process)
        return process

    def _wait(self, condition, timeout, description):
        until = time.monotonic() + timeout
        while not condition():
            if self.stop_event.wait(.1):
                raise InterruptedError('RViz 已停止')
            if any(p.poll() is not None for p in self.processes):
                raise RuntimeError(f'{description}失败；请查看本地 RViz 日志。')
            if time.monotonic() > until:
                raise RuntimeError(f'{description}超时。')

    @staticmethod
    def _listening(port):
        try:
            with socket.create_connection(('127.0.0.1', port), timeout=.2):
                return True
        except OSError:
            return False

    def _run(self, state):
        failure = ''
        cache = Path(os.environ.get('XDG_CACHE_HOME', Path.home() / '.cache')) / 'rebotarm-studio'
        cache.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='rebotarm-rviz-') as temporary, (cache / 'rviz-web.log').open('w') as logfile:
            try:
                directory = Path(temporary)
                # Authentication applies to both X11 and VNC; neither socket is public.
                display_number = next(n for n in range(120, 220) if not Path(f'/tmp/.X11-unix/X{n}').exists() and not Path(f'/tmp/.X{n}-lock').exists())
                display = f':{display_number}'
                auth = directory / 'Xauthority'
                subprocess.run(['xauth', '-f', str(auth)], input=f'add {display} . {secrets.token_hex(16)}\n'.encode(), stdout=logfile, stderr=logfile, check=True)
                auth.chmod(0o600)
                env = os.environ.copy()
                env.update(DISPLAY=display, XAUTHORITY=str(auth), QT_QPA_PLATFORM='xcb', REBOTARM_SAFETY_NAMESPACE=state['namespace'])
                managed = state.get('runtime', {}).get('managed', False)
                if managed:
                    env['ROS_DOMAIN_ID'] = str(state['runtime']['domain_id'])
                env['LD_LIBRARY_PATH'] = str(RUNTIME / 'usr/lib/x86_64-linux-gnu') + os.pathsep + env.get('LD_LIBRARY_PATH', '')
                env['PYTHONPATH'] = str(RUNTIME / 'usr/lib/python3/dist-packages') + os.pathsep + env.get('PYTHONPATH', '')
                env['XDG_RUNTIME_DIR'] = str(directory / 'xdg')
                Path(env['XDG_RUNTIME_DIR']).mkdir(mode=0o700)
                self._spawn([binary('Xvfb'), display, '-screen', '0', '1280x800x24', '-nolisten', 'tcp', '-auth', str(auth), '-noreset'], env, logfile)
                self._wait(lambda: Path(f'/tmp/.X11-unix/X{display_number}').exists(), 8, '虚拟显示启动')
                self.password = secrets.token_urlsafe(8)[:8]
                password_file = directory / 'vnc.passwd'
                library_path = RUNTIME / 'usr/lib/x86_64-linux-gnu/libvncserver.so.1'
                library = ctypes.CDLL(str(library_path) if library_path.exists() else 'libvncserver.so.1')
                library.rfbEncryptAndStorePasswd.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
                library.rfbEncryptAndStorePasswd.restype = ctypes.c_int
                if library.rfbEncryptAndStorePasswd(self.password.encode(), os.fsencode(password_file)) != 0:
                    raise RuntimeError('VNC 会话认证初始化失败。')
                password_file.chmod(0o600)
                vnc_port, self.ws_port = available_port(), available_port()
                self._spawn([binary('x11vnc'), '-display', display, '-auth', str(auth), '-localhost', '-rfbport', str(vnc_port),
                             '-rfbauth', str(password_file), '-forever', '-shared', '-noxdamage', '-wait', '16', '-defer', '16', '-quiet'], env, logfile)
                self._wait(lambda: self._listening(vnc_port), 10, 'VNC 启动')
                self._spawn(['/usr/bin/python3', '-m', 'websockify', '--auth-plugin', 'ExpectOrigin', '--auth-source', ' '.join(self.origins),
                             f'127.0.0.1:{self.ws_port}', f'127.0.0.1:{vnc_port}'], env, logfile)
                self._wait(lambda: self._listening(self.ws_port), 8, '浏览器显示通道启动')
                model = state['model'].upper()
                description = ROOT / f'src/rebotarm_bringup/description/{model}/urdf/ReBot_Arm_{model}.urdf'
                compat = ROOT / 'src/rebotarm_bringup/launch/rviz_urdf_compat.py'
                adapted = subprocess.run(['/usr/bin/python3', str(compat), str(description)], capture_output=True, text=True, check=True).stdout
                # Resolve mesh assets locally; this is display-only URDF, never a hardware launch.
                adapted = adapted.replace('package://rebotarm_bringup/', (ROOT / 'src/rebotarm_bringup').as_uri() + '/')
                import yaml
                params = {'/**': {'ros__parameters': {'robot_description': adapted, 'publish_frequency': 50.0}}}
                params_file = directory / 'robot.yaml'
                params_file.write_text(yaml.safe_dump(params))
                self._spawn(['/opt/ros/' + os.environ.get('ROS_DISTRO', 'jazzy') + '/lib/robot_state_publisher/robot_state_publisher',
                             '--ros-args', '--params-file', str(params_file), '-r', f'joint_states:=/{state["namespace"]}/joint_states',
                             '-r', 'robot_description:=/rebotarm_web_rviz/robot_description',
                             '-r', '/tf:=/rebotarm_web_rviz/tf', '-r', '/tf_static:=/rebotarm_web_rviz/tf_static',
                             '-r', '__node:=web_rviz_state_publisher'], env, logfile)
                config = yaml.safe_load((ROOT / 'src/rebotarm_bringup/rviz/rebotarm.rviz').read_text())
                # Keep the project model; only expose its motion panel when bridge motion is authorized.
                config['Panels'] = ([{'Class': 'rebotarm_rviz_panel/SafetyPanel', 'Name': 'Arm Safety',
                                      'Arm Namespace': state['namespace']}] if state.get('motion_allowed') else [])
                config['Panels'].append({'Class': 'rviz_common/Displays', 'Name': 'Displays'})
                manager = config['Visualization Manager']
                manager['Global Options']['Frame Rate'] = 30
                for item in manager['Displays']:
                    if item.get('Class') == 'rviz_default_plugins/RobotModel':
                        item['Description Topic']['Value'] = '/rebotarm_web_rviz/robot_description'
                        item['Description Topic']['Durability Policy'] = 'Transient Local'
                    if item.get('Name') == 'Voice Subtitles':
                        item['Topic']['Value'] = f'/{state["namespace"]}/voice/subtitles'
                manager['Views']['Current'].update(Distance=1.2, Pitch=.35, Yaw=.8)
                manager['Views']['Current']['Focal Point'] = {'X': .16, 'Y': 0, 'Z': .23}
                config['Window Geometry'].update(Width=1280, Height=800, X=0, Y=0)
                config['Window Geometry'].pop('QMainWindow State', None)
                config_file = directory / 'studio.rviz'
                config_file.write_text(yaml.safe_dump(config, sort_keys=False))
                rviz = self._spawn(['/opt/ros/' + os.environ.get('ROS_DISTRO', 'jazzy') + '/lib/rviz2/rviz2', '-d', str(config_file),
                                    '--ros-args', '-r', '/tf:=/rebotarm_web_rviz/tf',
                                    '-r', '/tf_static:=/rebotarm_web_rviz/tf_static'], env, logfile)
                # Qt/Ogre initialize asynchronously; fail early if the window cannot start.
                self._wait(lambda: subprocess.run(['xwininfo', '-display', display, '-root', '-tree'], env=env, capture_output=True, text=True).stdout.find('RViz') >= 0, 25, 'RViz 窗口启动')
                with self.lock:
                    self.status = 'running'
                while not self.stop_event.wait(.5):
                    if any(p.poll() is not None for p in self.processes):
                        raise RuntimeError('RViz 显示进程已退出，请重新连接。')
            except InterruptedError:
                pass
            except Exception as exc:
                failure = str(exc)
                print(f'RViz web: {failure}', flush=True)
            finally:
                for process in reversed(self.processes):
                    if process.poll() is None:
                        process.terminate()
                for process in reversed(self.processes):
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2)
                self.processes.clear()
                with self.lock:
                    self.status, self.error = ('error', failure) if failure else ('stopped', '')
                    self.password = None
                    self.ws_port = None

    def stop(self):
        with self.lock:
            if self.status in ('starting', 'running'):
                self.status = 'stopping'
                self.stop_event.set()
            return self.state()

    def close(self):
        self.stop()
        worker = self.worker
        if worker:
            worker.join(timeout=20)
