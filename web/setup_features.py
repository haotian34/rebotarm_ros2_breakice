#!/usr/bin/env python3
"""Install the optional MCP dependency in the project venv only."""
from pathlib import Path
import subprocess
import sys
import venv


def main():
    directory=Path(__file__).resolve().parent/'.feature-venv'
    if sys.version_info[:2] != (3,12):
        raise SystemExit('Use /usr/bin/python3 (ROS Jazzy Python 3.12).')
    venv.EnvBuilder(system_site_packages=True,with_pip=False).create(directory)
    subprocess.run([str(directory/'bin/python'),'-m','pip','install','-r',str(Path(__file__).with_name('requirements-features.txt'))],check=True)
    subprocess.run([str(directory/'bin/python'),'-c','import fastmcp; print("Local interaction dependencies ready")'],check=True)


if __name__=='__main__':
    main()
