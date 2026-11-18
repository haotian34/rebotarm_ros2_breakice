"""Render the very same state machines used by the original ROS action nodes.

The web owns no alternate choreography. Configs/IK frames live in their original
packages. This read-only preview adapter loads only the fixed pure methods from
trusted local node source; actual motion runs the unchanged original ROS nodes.
"""
from functools import lru_cache
import ast
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
JOINTS = [f'joint{i}' for i in range(1, 7)]
SDK_LIMITS = [5., 5., 5., 3., 3., 3.]


def load_motion(package, name, cls):
    path = ROOT / 'src' / package / package / f'{name}_node.py'
    original = next(node for node in ast.parse(path.read_text()).body
                    if isinstance(node, ast.ClassDef) and node.name == cls)
    selected = {'_tick'} if name == 'trajectory_player' else {'_build_steps', '_tick', '_run_step', '_clamp'}
    methods = [node for node in original.body if isinstance(node, ast.FunctionDef) and node.name in selected]
    # The paths and method whitelist above are fixed in repository code. No
    # request content, user-uploaded code or expressions can enter this compiler.
    definition = ast.ClassDef(name='OriginalMotion', bases=[], keywords=[], body=methods, decorator_list=[])
    module = ast.fix_missing_locations(ast.Module(body=[definition], type_ignores=[]))
    namespace = {'math': math, 'JOINTS': JOINTS}
    exec(compile(module, str(path), 'exec'), namespace)
    return namespace['OriginalMotion']


GreetingMotion = load_motion('rebotarm_greeting', 'greeting', 'GreetingNode')
DanceMotion = load_motion('rebotarm_dance', 'dance', 'DanceNode')
TrajectoryMotion = load_motion('rebotarm_gesture_actions', 'trajectory_player', 'TrajectoryPlayerNode')


@lru_cache(maxsize=12)
def _yaml(path, modified):
    import yaml
    return yaml.safe_load(Path(path).read_text())


def config(package, filename):
    path = ROOT / 'src' / package / 'config' / filename
    return _yaml(str(path), path.stat().st_mtime_ns)


class Capture:
    def __init__(self, positions, opening):
        self.current = dict(zip(JOINTS, positions))
        self.commanded = dict(self.current)
        self.step_from = dict(self.current)
        self.opening = opening
        self.gripper_command = False
        self.velocities = [0.] * 6
        self.gripper_vlim = 0.
        self.step_index = self.step_t = 0

    def _command(self, joint, pos):
        if hasattr(self, 'cfg'):
            lo, hi = self.cfg['joint_limits'][joint]
            limit = self.joint_vlim
        else:
            lo, hi = [(-2.8, 2.8), (-3.14, 0), (-3.14, 0), (-1.87, 1.57), (-1.57, 1.57), (-3.14, 3.14)][JOINTS.index(joint)]
            limit = self.vlim
            if self.state == 'TRACK':
                limit = {'rock_push': self.rock_vlim, 'paper_figure8': self.figure8_vlim}.get(self.track_name, limit)
        value = max(lo, min(hi, float(pos)))
        self.current[joint] = self.commanded[joint] = value
        self.velocities[JOINTS.index(joint)] = min(limit, SDK_LIMITS[JOINTS.index(joint)])

    def _command_gripper(self, pos):
        # Original DM scripts use negative motor positions; Fake uses positive.
        self.opening = max(0., min(90., -float(pos) * 18.))
        self.gripper_command = True
        self.gripper_vlim = min(3., float(self.cfg['gripper']['speed']))

    def _publish_status(self, text):
        pass

    def get_logger(self):
        return self

    def info(self, text):
        pass


class Greeting(Capture, GreetingMotion):
    pass


class Dance(Capture, DanceMotion):
    pass


class Track(Capture, TrajectoryMotion):
    pass


def render_program(action, positions, speed, opening=0):
    """Speed is global time scaling (1 = CLI defaults), never frame skipping."""
    if action in ('greeting', 'dance'):
        package = f'rebotarm_{action}'
        cfg = config(package, f'{action}.yaml')[action]
        player = (Greeting if action == 'greeting' else Dance)(positions, opening)
        player.cfg = cfg
        player.time_scale = float(cfg['speed']['time_scale'])
        player.joint_vlim = float(cfg['speed']['joint_vlim'])
        player.rate_hz = 30. if action == 'greeting' else 60.
        if action == 'dance':
            tracks = config(package, 'dance_tracks.yaml')
            player.tracks = {name: {'rate': float(tracks['rate_hz']), 'frames': item['frames']} for name, item in tracks['tracks'].items()}
            # dance.launch.py default; dance_node alone defaults to 1.5.
            player.track_speed = 3.
            player.track_ramp = float(cfg['speed']['track_ramp_s'])
        player.steps = player._build_steps()
        player.state = 'RUNNING'
        total_steps = len(player.steps)
    else:
        name = {'rock': 'rock_push', 'paper': 'paper_figure8', 'heart': 'heart_draw', 'circle': 'paper_circle'}[action]
        tracks = config('rebotarm_gesture_actions', 'gesture_tracks.yaml')
        player = Track(positions, opening)
        player.rate_hz = 30.
        player.entry_duration, player.reset_duration = 1.5, 2.
        player.vlim = player.rock_vlim = player.figure8_vlim = 5.
        player.reset_pose, player.reset_target = 'zeros', [0.] * 6
        player.frames = tracks['tracks'][name]['frames']
        player.frame_index, player.entry_t = 0, 0.
        player.entry_from = dict(player.current)
        player.track_name, player.state = name, 'ENTRY'
        total_steps = 3
    player.dt = 1. / player.rate_hz
    frames = []
    while player.state != 'IDLE':
        phase = player.state
        step = player.step_index + 1 if action in ('greeting', 'dance') else {'ENTRY': 1, 'TRACK': 2, 'RESET': 3}[phase]
        if action in ('greeting', 'dance') and player.step_index < total_steps:
            phase = player.steps[player.step_index]['type']
        player.gripper_command = False
        player._tick()
        frames.append({'joints': [player.current[j] for j in JOINTS], 'gripper': player.opening,
                       'gripper_command': player.gripper_command, 'gripper_vlim': player.gripper_vlim * speed,
                       'vlim': [v * speed for v in player.velocities],
                       'duration': player.dt / speed, 'step': min(step, total_steps), 'phase': phase})
        if len(frames) > 100000:
            raise ValueError('Original action exceeds the playback limit')
    return {'frames': frames, 'duration': len(frames) * player.dt / speed, 'total_steps': total_steps}
