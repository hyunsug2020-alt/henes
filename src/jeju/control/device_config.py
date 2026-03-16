#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import tempfile
from collections import defaultdict
from pathlib import Path
from typing import Any

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - available in sourced ROS env
    get_package_share_directory = None


class DeviceConfigError(RuntimeError):
    pass


def default_device_config_path() -> str:
    candidates: list[Path] = []

    env_path = os.environ.get('HENES_DEVICE_CONFIG', '').strip()
    if env_path:
        candidates.append(Path(env_path).expanduser())

    if get_package_share_directory is not None:
        try:
            candidates.append(Path(get_package_share_directory('jeju')) / 'config' / 'device_aliases.yaml')
        except Exception:
            pass

    candidates.append(Path('/home/coss/henes_ws_ros2/src/jeju/config/device_aliases.yaml'))

    for candidate in candidates:
        if candidate.exists():
            return str(candidate)

    return str(candidates[0])


def load_device_config(config_file: str) -> dict[str, Any]:
    path = Path(config_file).expanduser()
    if not path.exists():
        raise DeviceConfigError(f'device config not found: {path}')

    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    devices = data.get('devices')
    if not isinstance(devices, list):
        raise DeviceConfigError(f'invalid device config: {path} (devices must be a list)')

    normalized: list[dict[str, Any]] = []
    seen_names: set[str] = set()

    for raw in devices:
        if not isinstance(raw, dict):
            raise DeviceConfigError(f'invalid device entry in {path}: {raw!r}')

        name = str(raw.get('name', '')).strip()
        role = str(raw.get('role', '')).strip()
        symlink = str(raw.get('symlink', '')).strip()
        if not name or not role or not symlink:
            raise DeviceConfigError(
                f'invalid device entry in {path}: name/role/symlink are required ({raw!r})'
            )
        if name in seen_names:
            raise DeviceConfigError(f'duplicate device name in {path}: {name}')
        seen_names.add(name)

        extra_symlinks = raw.get('extra_symlinks') or []
        if isinstance(extra_symlinks, str):
            extra_symlinks = [extra_symlinks]
        if not isinstance(extra_symlinks, list):
            raise DeviceConfigError(f'invalid extra_symlinks for {name} in {path}')

        normalized.append({
            'name': name,
            'role': role,
            'symlink': symlink,
            'extra_symlinks': [str(item).strip() for item in extra_symlinks if str(item).strip()],
            'default_for_role': bool(raw.get('default_for_role', False)),
            'family': str(raw.get('family', '')).strip(),
            'notes': str(raw.get('notes', '')).strip(),
            'match': raw.get('match') or {},
        })

    detection_rules = data.get('detection_rules') or []
    if not isinstance(detection_rules, list):
        raise DeviceConfigError(f'invalid detection_rules in {path}')

    normalized_rules: list[dict[str, Any]] = []
    for raw in detection_rules:
        if not isinstance(raw, dict):
            raise DeviceConfigError(f'invalid detection rule in {path}: {raw!r}')
        role = str(raw.get('role', '')).strip()
        if not role:
            raise DeviceConfigError(f'detection rule missing role in {path}: {raw!r}')
        normalized_rules.append({
            'role': role,
            'family': str(raw.get('family', '')).strip(),
            'match': raw.get('match') or {},
        })

    return {
        'path': str(path),
        'devices': normalized,
        'detection_rules': normalized_rules,
    }


def get_device_path(device: dict[str, Any]) -> str:
    return f"/dev/{device['symlink']}"


def get_device_by_name(config: dict[str, Any], name: str) -> dict[str, Any]:
    for device in config['devices']:
        if device['name'] == name:
            return device
    raise DeviceConfigError(f'unknown device name: {name}')


def get_role_devices(config: dict[str, Any], role: str) -> list[dict[str, Any]]:
    return [device for device in config['devices'] if device['role'] == role]


def get_default_device(config: dict[str, Any], role: str) -> dict[str, Any]:
    role_devices = get_role_devices(config, role)
    if not role_devices:
        raise DeviceConfigError(f'no devices configured for role: {role}')

    for device in role_devices:
        if device['default_for_role']:
            return device
    return role_devices[0]


def _has_complete_match(match: dict[str, Any] | None) -> bool:
    match = match or {}
    method = str(match.get('method', '')).strip().lower()
    if method == 'serial':
        return not any(
            _is_placeholder(str(match.get(key, '')).strip())
            for key in ('vendor_id', 'product_id', 'serial_short')
        )
    if method == 'id_path':
        return not _is_placeholder(str(match.get('id_path', '')).strip())
    return False


def _attached_matches_config_device(attached_device: dict[str, str], config_device: dict[str, Any]) -> bool:
    match = config_device.get('match') or {}
    method = str(match.get('method', '')).strip().lower()
    if not _has_complete_match(match):
        return False

    if method == 'serial':
        return (
            attached_device.get('vendor_id', '').strip() == str(match.get('vendor_id', '')).strip()
            and attached_device.get('product_id', '').strip() == str(match.get('product_id', '')).strip()
            and attached_device.get('serial_short', '').strip() == str(match.get('serial_short', '')).strip()
        )

    if method == 'id_path':
        return attached_device.get('id_path', '').strip() == str(match.get('id_path', '')).strip()

    return False


def _find_attached_device_for_config_device(config_device: dict[str, Any]) -> dict[str, str] | None:
    if not _has_complete_match(config_device.get('match')):
        return None

    for attached_device in list_serial_devices():
        if _attached_matches_config_device(attached_device, config_device):
            return attached_device

    return None


def resolve_device_selection(
    config: dict[str, Any],
    role: str,
    requested_name: str = '',
    requested_path: str = '',
) -> dict[str, Any]:
    requested_name = requested_name.strip()
    requested_path = requested_path.strip()

    if requested_path:
        return {
            'role': role,
            'source': 'path',
            'name': requested_name,
            'path': requested_path,
            'alias_path': requested_path,
            'path_source': 'path',
            'matched_device': None,
            'device': None,
        }

    if requested_name:
        device = get_device_by_name(config, requested_name)
        if device['role'] != role:
            raise DeviceConfigError(
                f'device name {requested_name} is role={device["role"]}, expected role={role}'
            )
        source = 'name'
    else:
        device = get_default_device(config, role)
        source = 'default'

    alias_path = get_device_path(device)
    matched_device = None
    path = alias_path
    path_source = 'symlink'

    if not Path(alias_path).exists():
        matched_device = _find_attached_device_for_config_device(device)
        if matched_device is not None:
            path = matched_device['device']
            path_source = 'live_match'
        else:
            path_source = 'missing_symlink'

    return {
        'role': role,
        'source': source,
        'name': device['name'],
        'path': path,
        'alias_path': alias_path,
        'path_source': path_source,
        'matched_device': matched_device,
        'device': device,
    }


def format_device_resolution(selection: dict[str, Any], label: str) -> str:
    if selection['source'] == 'path':
        return f'{label} resolved from explicit path -> {selection["path"]}'

    message = (
        f'{label} resolved from YAML '
        f'({selection["source"]}: {selection["name"]}) -> {selection["path"]}'
    )
    path_source = selection.get('path_source', 'path')
    if path_source == 'live_match':
        message += f' [live-match fallback, alias missing: {selection.get("alias_path", "")}]'
    elif path_source == 'missing_symlink':
        message += f' [alias missing: {selection.get("alias_path", "")}]'
    return message


def resolve_device_path(
    config_file: str,
    role: str,
    requested_name: str = '',
    requested_path: str = '',
) -> str:
    config = load_device_config(config_file)
    selection = resolve_device_selection(
        config,
        role,
        requested_name=requested_name,
        requested_path=requested_path,
    )
    return selection['path']


def _is_placeholder(value: str) -> bool:
    value = value.strip()
    return (not value) or ('REPLACE_ME' in value)


def build_udev_rule(device: dict[str, Any]) -> str:
    match = device.get('match') or {}
    method = str(match.get('method', '')).strip().lower()
    if not method:
        raise DeviceConfigError(f'missing match.method for {device["name"]}')

    if method == 'serial':
        vendor_id = str(match.get('vendor_id', '')).strip()
        product_id = str(match.get('product_id', '')).strip()
        serial_short = str(match.get('serial_short', '')).strip()
        if _is_placeholder(vendor_id) or _is_placeholder(product_id) or _is_placeholder(serial_short):
            raise DeviceConfigError(
                f'incomplete serial match for {device["name"]}: vendor_id/product_id/serial_short required'
            )
        selector = (
            f'ATTRS{{idVendor}}=="{vendor_id}", '
            f'ATTRS{{idProduct}}=="{product_id}", '
            f'ATTRS{{serial}}=="{serial_short}"'
        )
    elif method == 'id_path':
        id_path = str(match.get('id_path', '')).strip()
        if _is_placeholder(id_path):
            raise DeviceConfigError(f'incomplete id_path match for {device["name"]}')
        selector = f'ENV{{ID_PATH}}=="{id_path}"'
    else:
        raise DeviceConfigError(f'unsupported match.method for {device["name"]}: {method}')

    symlink_parts = [device['symlink'], *device.get('extra_symlinks', [])]
    symlink_expr = ', '.join(f'SYMLINK+="{item}"' for item in symlink_parts)
    return f'SUBSYSTEM=="tty", {selector}, {symlink_expr}, MODE="0666"'


def find_conflicting_udev_rule_files(dest: str = '/etc/udev/rules.d/99-henes-serial.rules') -> list[str]:
    rules_dir = Path('/etc/udev/rules.d')
    if not rules_dir.exists():
        return []

    dest_name = Path(dest).name
    return [
        str(candidate)
        for candidate in sorted(rules_dir.glob('99-henes*'))
        if candidate.name != dest_name and candidate.suffix == '.rules'
    ]


def render_udev_rules(config: dict[str, Any]) -> tuple[list[str], list[str]]:
    rules: list[str] = []
    warnings: list[str] = []

    for device in config['devices']:
        try:
            rules.append(build_udev_rule(device))
        except DeviceConfigError as exc:
            warnings.append(str(exc))

    for conflict in find_conflicting_udev_rule_files():
        warnings.append(f'legacy/conflicting udev rule present: {conflict}')

    return rules, warnings


def _run(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, capture_output=True, text=True, check=False)


def _get_props(dev: str) -> dict[str, str]:
    out = _run(['udevadm', 'info', '-q', 'property', '-n', dev])
    if out.returncode != 0:
        return {}

    props: dict[str, str] = {}
    for line in out.stdout.splitlines():
        if '=' in line:
            key, value = line.split('=', 1)
            props[key.strip()] = value.strip()
    return props


def _find_by_id_link(dev: str) -> str:
    by_id_dir = Path('/dev/serial/by-id')
    if not by_id_dir.exists():
        return ''

    for link in sorted(by_id_dir.iterdir()):
        try:
            if str(link.resolve()) == dev:
                return str(link)
        except Exception:
            continue
    return ''


def list_serial_devices() -> list[dict[str, str]]:
    out = _run(['bash', '-lc', 'ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | sort -V'])
    if out.returncode != 0 or not out.stdout.strip():
        return []

    devices: list[dict[str, str]] = []
    for dev in [line.strip() for line in out.stdout.splitlines() if line.strip()]:
        props = _get_props(dev)
        devices.append({
            'device': dev,
            'vendor_id': props.get('ID_VENDOR_ID', ''),
            'product_id': props.get('ID_MODEL_ID', ''),
            'vendor': props.get('ID_VENDOR', ''),
            'model': props.get('ID_MODEL', ''),
            'serial': props.get('ID_SERIAL', ''),
            'vendor_product': f"{props.get('ID_VENDOR_ID', '?')}:{props.get('ID_MODEL_ID', '?')}",
            'serial_short': props.get('ID_SERIAL_SHORT', ''),
            'id_path': props.get('ID_PATH', ''),
            'by_id': _find_by_id_link(dev),
        })
    return devices


def save_device_config(config: dict[str, Any], config_file: str | None = None) -> None:
    path = Path(config_file or config['path']).expanduser()
    body = {
        'devices': config['devices'],
        'detection_rules': config.get('detection_rules', []),
    }
    path.write_text(yaml.safe_dump(body, sort_keys=False, allow_unicode=False), encoding='utf-8')


def _rule_value(props: dict[str, str], field: str) -> str:
    mapping = {
        'vendor_id': props.get('vendor_id', ''),
        'product_id': props.get('product_id', ''),
        'vendor': props.get('vendor', ''),
        'model': props.get('model', ''),
        'serial': props.get('serial', ''),
        'serial_short': props.get('serial_short', ''),
        'id_path': props.get('id_path', ''),
    }
    return mapping.get(field, '')


def _matches_detection_rule(device: dict[str, str], rule: dict[str, Any]) -> bool:
    match = rule.get('match') or {}
    for key, expected in match.items():
        actual = _rule_value(device, str(key))
        if key.endswith('_regex'):
            field = key[:-6]
            actual = _rule_value(device, field)
            if not re.search(str(expected), actual):
                return False
            continue
        if str(actual).strip() != str(expected).strip():
            return False
    return True


def _name_sort_key(name: str) -> tuple[str, int, str]:
    match = re.match(r'^([a-zA-Z_]+?)(\d+)$', name)
    if match:
        return match.group(1), int(match.group(2)), name
    return name, 0, name


def _build_match_block(attached_device: dict[str, str]) -> dict[str, str]:
    if attached_device.get('serial_short'):
        return {
            'method': 'serial',
            'vendor_id': attached_device.get('vendor_id', ''),
            'product_id': attached_device.get('product_id', ''),
            'serial_short': attached_device.get('serial_short', ''),
        }
    return {
        'method': 'id_path',
        'id_path': attached_device.get('id_path', ''),
    }


def autofill_device_matches(config: dict[str, Any]) -> tuple[list[str], list[str]]:
    attached = list_serial_devices()
    rules = config.get('detection_rules', [])
    if not rules:
        raise DeviceConfigError('no detection_rules configured')

    matched_by_role: dict[str, list[dict[str, str]]] = defaultdict(list)
    warnings: list[str] = []
    changes: list[str] = []

    for attached_device in attached:
        matched_rule = None
        for rule in rules:
            if _matches_detection_rule(attached_device, rule):
                matched_rule = rule
                break
        if matched_rule is None:
            warnings.append(
                f"unmatched device: {attached_device['device']} "
                f"({attached_device['vendor_product']} {attached_device['model']})"
            )
            continue
        matched_by_role[matched_rule['role']].append(attached_device)

    for role, attached_devices in matched_by_role.items():
        config_devices = sorted(get_role_devices(config, role), key=lambda item: _name_sort_key(item['name']))
        attached_devices = sorted(
            attached_devices,
            key=lambda item: (
                item.get('serial_short', ''),
                item.get('id_path', ''),
                item.get('device', ''),
            ),
        )

        assigned_pairs: list[tuple[dict[str, Any], dict[str, str]]] = []
        remaining_config_devices: list[dict[str, Any]] = []
        remaining_attached_devices = list(attached_devices)

        # Preserve existing name -> device assignments when the current match block
        # already points to an attached device.
        for config_device in config_devices:
            matched_device = next(
                (
                    candidate
                    for candidate in remaining_attached_devices
                    if _attached_matches_config_device(candidate, config_device)
                ),
                None,
            )
            if matched_device is None:
                remaining_config_devices.append(config_device)
                continue
            assigned_pairs.append((config_device, matched_device))
            remaining_attached_devices.remove(matched_device)

        if len(attached_devices) > len(config_devices):
            warnings.append(
                f'role={role}: attached {len(attached_devices)} device(s), '
                f'but config has only {len(config_devices)} slot(s)'
            )

        for index, attached_device in enumerate(remaining_attached_devices[:len(remaining_config_devices)]):
            assigned_pairs.append((remaining_config_devices[index], attached_device))

        for config_device, attached_device in assigned_pairs:
            match_block = _build_match_block(attached_device)
            changed = config_device.get('match') != match_block
            config_device['match'] = match_block
            if changed:
                changes.append(
                    f"{config_device['name']} -> {attached_device['device']} "
                    f"({attached_device.get('model', '')} {attached_device.get('serial_short', '')})"
                )

    return changes, warnings


def install_udev_rules(config: dict[str, Any], dest: str) -> tuple[list[str], list[str]]:
    rules, warnings = render_udev_rules(config)
    if not rules:
        raise DeviceConfigError('no valid udev rules to install')

    body = '\n'.join(rules) + '\n'
    with tempfile.NamedTemporaryFile('w', encoding='utf-8', delete=False, suffix='.rules') as tmp:
        tmp.write(body)
        tmp_path = tmp.name

    copy_result = _run(['sudo', 'cp', tmp_path, dest])
    if copy_result.returncode != 0:
        raise DeviceConfigError(copy_result.stderr.strip() or f'failed to copy rule file to {dest}')

    reload_result = _run(['sudo', 'udevadm', 'control', '--reload-rules'])
    if reload_result.returncode != 0:
        raise DeviceConfigError(reload_result.stderr.strip() or 'failed to reload udev rules')

    trigger_result = _run(['sudo', 'udevadm', 'trigger'])
    if trigger_result.returncode != 0:
        raise DeviceConfigError(trigger_result.stderr.strip() or 'failed to trigger udev')

    return rules, warnings


def _print_config(config: dict[str, Any]) -> int:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for device in config['devices']:
        grouped[device['role']].append(device)

    print(f"config: {config['path']}")
    for role in sorted(grouped):
        print(f'\n[{role}]')
        for device in grouped[role]:
            extra = ''
            if device.get('extra_symlinks'):
                extra = f" (+ {', '.join('/dev/' + item for item in device['extra_symlinks'])})"
            default_text = ' default' if device.get('default_for_role') else ''
            print(f"  {device['name']}: {get_device_path(device)}{extra}{default_text}")
    return 0


def _print_device_list() -> int:
    devices = list_serial_devices()
    if not devices:
        print('연결된 /dev/ttyUSB* 또는 /dev/ttyACM* 장치가 없습니다.')
        return 1

    print('No | Device       | Vendor:Product | Serial            | ID_PATH                          | /dev/serial/by-id')
    print('-' * 120)
    for index, device in enumerate(devices, start=1):
        print(
            f"{index:>2} | {device['device']:<12} | {device['vendor_product']:<14} | "
            f"{device['serial_short'] or 'N/A':<17} | {device['id_path'] or 'N/A':<32} | "
            f"{device['by_id'] or 'N/A'}"
        )
    return 0


def _print_rules(config: dict[str, Any]) -> int:
    rules, warnings = render_udev_rules(config)
    for rule in rules:
        print(rule)
    if warnings:
        print('\n# warnings')
        for warning in warnings:
            print(f'# {warning}')
    return 0 if rules else 1


def _print_resolution(config: dict[str, Any], role: str, name: str, path: str) -> int:
    selection = resolve_device_selection(config, role, requested_name=name, requested_path=path)
    print(f"role={role}")
    print(f"source={selection['source']}")
    if selection['name']:
        print(f"name={selection['name']}")
    print(f"path_source={selection.get('path_source', 'path')}")
    if selection.get('alias_path'):
        print(f"alias_path={selection['alias_path']}")
    print(f"path={selection['path']}")
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='HENES serial device config and udev rule tool',
    )
    parser.add_argument('--config', default=default_device_config_path())

    subparsers = parser.add_subparsers(dest='command')

    subparsers.add_parser('show', help='show configured device names and paths')
    subparsers.add_parser('list', help='list currently attached ttyUSB/ttyACM devices')
    subparsers.add_parser('render', help='render udev rules from YAML config')

    install_parser = subparsers.add_parser('install', help='install udev rules from YAML config')
    install_parser.add_argument(
        '--dest',
        default='/etc/udev/rules.d/99-henes-serial.rules',
        help='destination udev rule file',
    )

    autofill_parser = subparsers.add_parser(
        'autofill',
        help='scan attached tty devices and update YAML match blocks automatically',
    )
    autofill_parser.add_argument(
        '--write',
        action='store_true',
        help='write detected matches back into the YAML file',
    )

    resolve_parser = subparsers.add_parser('resolve', help='resolve a role/name to an actual /dev path')
    resolve_parser.add_argument('--role', required=True, choices=['gps', 'imu', 'arduino', 'lidar'])
    resolve_parser.add_argument('--name', default='')
    resolve_parser.add_argument('--path', default='')

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    command = args.command or 'show'

    if command == 'list':
        return _print_device_list()

    config = load_device_config(args.config)

    if command == 'show':
        return _print_config(config)
    if command == 'render':
        return _print_rules(config)
    if command == 'resolve':
        return _print_resolution(config, args.role, args.name, args.path)
    if command == 'install':
        rules, warnings = install_udev_rules(config, args.dest)
        print(f'installed {len(rules)} rule(s) -> {args.dest}')
        if warnings:
            print('warnings:')
            for warning in warnings:
                print(f'  - {warning}')
        return 0
    if command == 'autofill':
        changes, warnings = autofill_device_matches(config)
        if changes:
            print('detected assignments:')
            for change in changes:
                print(f'  - {change}')
        else:
            print('no config entries changed')
        if warnings:
            print('warnings:')
            for warning in warnings:
                print(f'  - {warning}')
        if args.write:
            save_device_config(config, args.config)
            print(f'updated config: {args.config}')
        return 0

    parser.print_help()
    return 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except DeviceConfigError as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(1)
