"""Resolve startreal's channel without opening or enabling any hardware."""

import os
from pathlib import Path
import sys

import yaml


def select_channel(default_config, arguments, environment=None, by_id=Path('/dev/serial/by-id')):
    environment = os.environ if environment is None else environment
    options = {}
    for argument in arguments:
        key, separator, value = argument.partition(':=')
        if separator:
            options[key] = value
    explicit = options.get('channel') or environment.get('REBOTARM_CHANNEL', '')
    if explicit:
        return explicit
    config_path = Path(options.get('hardware_config') or default_config).expanduser()
    config = yaml.safe_load(config_path.read_text()) or {}
    model = (options.get('model') or config.get('default_model') or 'dm').strip().lower()
    if model not in config.get('models', {}):
        raise ValueError(f'Unknown hardware model: {model}')
    configured = str(config['models'][model].get('overrides', {}).get('channel', ''))
    if model != 'dm' or configured not in ('', '/dev/ttyACM0'):
        if not configured:
            raise ValueError(f'No channel configured for model {model}')
        return configured
    candidates = {}
    for path in sorted(by_id.glob('*')):
        target = path.resolve()
        if target.name.startswith(('ttyACM', 'ttyUSB')) and target.is_char_device():
            candidates.setdefault(target, str(path))
    if len(candidates) == 1:
        return next(iter(candidates.values()))
    if len(candidates) > 1:
        raise ValueError(
            f'Multiple USB serial devices: {list(candidates.values())}. '
            'Specify channel:=<device> or REBOTARM_CHANNEL.'
        )
    # Some adapters have no serial number and thus no by-id link. Accept a
    # unique current tty node, but never guess among multiple devices.
    devices = sorted(set(Path('/dev').glob('ttyACM*')) | set(Path('/dev').glob('ttyUSB*')))
    devices = [path for path in devices if path.is_char_device()]
    if len(devices) == 1:
        return str(devices[0])
    raise ValueError(
        f'Cannot choose a USB serial device: {[str(path) for path in devices]}. '
        'Check the adapter connection or specify channel:=<device>.'
    )


if __name__ == '__main__':
    try:
        print(select_channel(sys.argv[1], sys.argv[2:]))
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f'Hardware channel selection failed: {error}', file=sys.stderr)
        sys.exit(1)
