#!/usr/bin/env python3
"""Install the local studio as a user service and application shortcut."""
import argparse
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
SERVICE_NAME = 'rebotarm-studio.service'
APP_NAME = 'rebotarm-studio.desktop'


def quoted(value, systemd=False):
    value = str(value).replace('\\', '\\\\').replace('"', '\\"')
    return '"' + (value.replace('%', '%%') if systemd else value) + '"'


def files():
    config = Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config'))
    data = Path(os.environ.get('XDG_DATA_HOME', Path.home() / '.local/share'))
    service = f'''[Unit]
Description=reBot Arm Studio local simulation and DM control webpage

[Service]
Type=simple
WorkingDirectory={str(ROOT).replace('%', '%%')}
ExecStart=/usr/bin/bash {quoted(ROOT / 'web/launch_studio.sh', True)}
Environment=ROS_LOG_DIR=%C/rebotarm-studio/ros
CacheDirectory=rebotarm-studio
Restart=on-failure
RestartSec=3
KillSignal=SIGINT
TimeoutStopSec=45

[Install]
WantedBy=default.target
'''
    desktop = f'''[Desktop Entry]
Type=Application
Name=reBot Arm Studio
Name[zh_CN]=reBot Arm 仿真工作台
Comment=打开网页并自动启动本机 ROS 2 仿真
Exec=/usr/bin/bash {quoted(ROOT / 'web/open_studio.sh')}
Icon=applications-engineering
Terminal=false
Categories=Development;Education;
StartupNotify=false
'''
    result = [(config / 'systemd/user' / SERVICE_NAME, service, 0o644),
              (data / 'applications' / APP_NAME, desktop, 0o644)]
    directory = subprocess.run(['xdg-user-dir', 'DESKTOP'], capture_output=True, text=True, check=False).stdout.strip()
    if directory and Path(directory).is_dir() and Path(directory) != Path.home():
        result.append((Path(directory) / APP_NAME, desktop, 0o755))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--remove', action='store_true', help='Disable the service and remove these shortcuts')
    parser.add_argument('--preview', action='store_true', help='Print generated files without installing')
    args = parser.parse_args()
    entries = files()
    if args.preview:
        for path, content, _ in entries:
            print(f'{path}\n{content}')
        return
    if args.remove:
        subprocess.run(['systemctl', '--user', 'disable', '--now', SERVICE_NAME], check=True)
        for path, _, _ in entries:
            if path.exists():
                # Do not remove an unrelated file that has since replaced ours.
                if str(ROOT / 'web/') in path.read_text():
                    path.unlink()
        subprocess.run(['systemctl', '--user', 'daemon-reload'], check=True)
        print('已移除本机网页服务和快捷方式。')
        return
    for path, content, mode in entries:
        if path.exists() and str(ROOT / 'web/') not in path.read_text():
            raise SystemExit(f'保留已有文件，未覆盖：{path}')
    for path, content, mode in entries:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + '.tmp')
        temporary.write_text(content)
        temporary.chmod(mode)
        temporary.replace(path)
    subprocess.run(['systemctl', '--user', 'daemon-reload'], check=True)
    subprocess.run(['systemd-analyze', '--user', 'verify', str(entries[0][0])], check=True)
    subprocess.run(['systemctl', '--user', 'enable', SERVICE_NAME], check=True)
    subprocess.run(['systemctl', '--user', 'start', SERVICE_NAME], check=True)
    subprocess.run(['systemctl', '--user', 'is-active', '--quiet', SERVICE_NAME], check=True)
    for path, _, mode in entries:
        if mode == 0o755:
            subprocess.run(['gio', 'set', str(path), 'metadata::trusted', 'true'], check=False, capture_output=True)
    print('已安装本机仿真网页服务与应用/桌面快捷方式。')
    print('以后直接打开 http://127.0.0.1:8088 或双击 reBot Arm 仿真工作台。')


if __name__ == '__main__':
    main()
