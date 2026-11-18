#!/usr/bin/env python3
"""Install portable Ubuntu noVNC/Xvfb dependencies inside this project (no sudo)."""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parent
PACKAGES = ['xvfb', 'x11vnc', 'novnc', 'python3-websockify', 'libxfont2', 'libvncserver1', 'libvncclient1']


def main():
    runtime = ROOT / '.rviz-runtime'
    runtime.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='rebotarm-rviz-debs-') as directory:
        subprocess.run(['apt-get', 'download', *PACKAGES], cwd=directory, check=True)
        for package in Path(directory).glob('*.deb'):
            subprocess.run(['dpkg-deb', '-x', str(package), str(runtime)], check=True)
    vendor = ROOT / 'dist/vendor/novnc'
    vendor.mkdir(parents=True, exist_ok=True)
    for name in ('core', 'vendor'):
        shutil.copytree(runtime / 'usr/share/novnc' / name, vendor / name, dirs_exist_ok=True)
    shutil.copyfile(runtime / 'usr/share/doc/novnc/copyright', vendor / 'LICENSES.txt')
    print('RViz 网页依赖已安装到 web/.rviz-runtime，无需 sudo。')


if __name__ == '__main__':
    main()
