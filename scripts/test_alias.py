"""Shell launch regressions; no ROS processes or hardware are started."""
from pathlib import Path
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[1]


class SimulationAliasTest(unittest.TestCase):
    def run_shell(self, script):
        result = subprocess.run(
            ['bash', '--noprofile', '--norc', '-c', script], cwd=ROOT,
            capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result.stdout

    def test_replaces_existing_alias_and_preserves_parent_environment(self):
        output = self.run_shell(r'''
shopt -s expand_aliases
alias startsimulation='false'
source ./alias.txt
# Stub environment loading and the final command; no ROS launch occurs.
loadros() { :; }
loadarm() { :; }
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/snap/core20/current/lib/x86_64-linux-gnu:/custom/lib
export GTK_PATH=/snap/code/current/gtk
export QT_PLUGIN_PATH=/var/lib/snapd/snap/code/current/plugins:/custom/plugins
ros2() {
    [[ "$LD_LIBRARY_PATH" == /opt/ros/jazzy/lib:/custom/lib ]] || return 11
    [[ ! -v GTK_PATH ]] || return 12
    [[ "$QT_PLUGIN_PATH" == /custom/plugins ]] || return 13
    printf '%s\n' "$@"
}
startsimulation arm_namespace:=test use_rviz:=false || exit
[[ "$LD_LIBRARY_PATH" == /opt/ros/jazzy/lib:/snap/core20/current/lib/x86_64-linux-gnu:/custom/lib ]] || exit 14
[[ "$GTK_PATH" == /snap/code/current/gtk ]] || exit 15
''')
        self.assertEqual(output.splitlines(), [
            'launch', 'rebotarm_bringup', 'fake_dm_bringup.launch.py',
            'use_rviz:=true', 'arm_namespace:=test', 'use_rviz:=false',
        ])

    def test_failed_environment_setup_does_not_launch(self):
        output = self.run_shell('''
source ./alias.txt
loadros() { return 7; }
ros2() { echo unexpected-launch; }
startsimulation
[[ $? == 7 ]]
''')
        self.assertEqual(output, '')


if __name__ == '__main__':
    unittest.main()
