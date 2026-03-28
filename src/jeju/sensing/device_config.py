#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HENES 시리얼 포트 고정 관리 도구
===================================
사용법:
  python3 device_config.py               # 현재 설정 표시 (show)
  python3 device_config.py list          # 연결된 장치 목록
  python3 device_config.py show          # 설정된 장치 및 심볼릭 링크 표시
  python3 device_config.py render        # udev rule 미리보기
  python3 device_config.py autofill      # 연결된 장치 자동 감지 (미리보기)
  python3 device_config.py autofill --write  # 자동 감지 후 YAML 저장
  python3 device_config.py install       # udev rule 설치 (/etc/udev/rules.d/)
  python3 device_config.py resolve --role gps  # 역할별 장치 경로 확인

설정 파일: ~/henes_device_config.yaml (없으면 내장 기본값 사용)
"""

from __future__ import annotations

import argparse, os, re, subprocess, sys, tempfile
from collections import defaultdict
from pathlib import Path
from typing import Any

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None


# ============================================================
# 내장 기본 설정 (YAML 파일 없을 때 사용)
# ============================================================

BUILTIN_CONFIG: dict = {
    "devices": [
        {
            "name": "gps1",
            "role": "gps",
            "symlink": "henes_gps",
            "extra_symlinks": ["henes_gps1"],
            "default_for_role": True,
            "family": "F9P",
            "notes": "Primary rover GPS.",
            "match": {
                "method": "serial",
                "vendor_id": "1546",
                "product_id": "0502",
                "serial_short": "DBTP8MS9",
            },
        },
        {
            "name": "imu1",
            "role": "imu",
            "symlink": "henes_imu",
            "extra_symlinks": ["henes_imu1"],
            "default_for_role": True,
            "family": "",
            "notes": "Primary IMU.",
            "match": {
                "method": "serial",
                "vendor_id": "10c4",
                "product_id": "ea60",
                "serial_short": "0001",
            },
        },
        {
            "name": "arduino1",
            "role": "arduino",
            "symlink": "henes_arduino",
            "extra_symlinks": ["henes_arduino1"],
            "default_for_role": True,
            "family": "",
            "notes": "Primary drive controller.",
            "match": {
                "method": "serial",
                "vendor_id": "2341",
                "product_id": "0042",
                "serial_short": "24235313336351513022",
            },
        },
    ],
    "detection_rules": [
        {"role": "gps",     "family": "F9P", "match": {"vendor_id": "1546", "model_regex": "ZED-F9P"}},
        {"role": "imu",     "family": "",    "match": {"vendor_id": "10c4", "product_id": "ea60"}},
        {"role": "arduino", "family": "",    "match": {"vendor_id": "2341", "product_id": "0042"}},
    ],
}


# ============================================================
# 설정 파일 경로
# ============================================================

def default_config_path() -> str:
    candidates = []

    env_path = os.environ.get('HENES_DEVICE_CONFIG', '').strip()
    if env_path:
        candidates.append(Path(env_path).expanduser())

    if get_package_share_directory is not None:
        try:
            candidates.append(
                Path(get_package_share_directory('jeju')) / 'config' / 'device_aliases.yaml'
            )
        except Exception:
            pass

    candidates += [
        Path('/home/coss/henes_ws_ros2/src/jeju/config/device_aliases.yaml'),
        Path.home() / 'henes_device_config.yaml',
    ]

    for c in candidates:
        if c.exists():
            return str(c)

    return str(Path.home() / 'henes_device_config.yaml')


# ============================================================
# 예외
# ============================================================

class DeviceConfigError(RuntimeError):
    pass


# ============================================================
# 설정 로드 / 저장
# ============================================================

def load_config(config_file: str) -> dict[str, Any]:
    path = Path(config_file).expanduser()

    if not path.exists():
        data = BUILTIN_CONFIG
    else:
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}

    devices = data.get('devices')
    if not isinstance(devices, list):
        raise DeviceConfigError(f'devices must be a list: {path}')

    normalized, seen = [], set()
    for raw in devices:
        name, role, symlink = str(raw.get('name','')).strip(), str(raw.get('role','')).strip(), str(raw.get('symlink','')).strip()
        if not name or not role or not symlink:
            raise DeviceConfigError(f'name/role/symlink required: {raw!r}')
        if name in seen:
            raise DeviceConfigError(f'duplicate name: {name}')
        seen.add(name)

        extra = raw.get('extra_symlinks') or []
        if isinstance(extra, str):
            extra = [extra]

        normalized.append({
            'name': name,
            'role': role,
            'symlink': symlink,
            'extra_symlinks': [str(i).strip() for i in extra if str(i).strip()],
            'default_for_role': bool(raw.get('default_for_role', False)),
            'family': str(raw.get('family', '')).strip(),
            'notes': str(raw.get('notes', '')).strip(),
            'match': raw.get('match') or {},
        })

    rules = data.get('detection_rules') or []
    normalized_rules = []
    for raw in rules:
        role = str(raw.get('role', '')).strip()
        if not role:
            raise DeviceConfigError(f'detection rule missing role: {raw!r}')
        normalized_rules.append({
            'role': role,
            'family': str(raw.get('family', '')).strip(),
            'match': raw.get('match') or {},
        })

    return {'path': str(path), 'devices': normalized, 'detection_rules': normalized_rules}


def save_config(config: dict[str, Any], config_file: str | None = None) -> None:
    path = Path(config_file or config['path']).expanduser()
    body = {'devices': config['devices'], 'detection_rules': config.get('detection_rules', [])}
    path.write_text(yaml.safe_dump(body, sort_keys=False, allow_unicode=False), encoding='utf-8')


# ============================================================
# 장치 조회 헬퍼
# ============================================================

def get_device_path(device: dict[str, Any]) -> str:
    return f"/dev/{device['symlink']}"

def get_device_by_name(config: dict[str, Any], name: str) -> dict[str, Any]:
    for d in config['devices']:
        if d['name'] == name:
            return d
    raise DeviceConfigError(f'unknown device: {name}')

def get_role_devices(config: dict[str, Any], role: str) -> list[dict[str, Any]]:
    return [d for d in config['devices'] if d['role'] == role]

def get_default_device(config: dict[str, Any], role: str) -> dict[str, Any]:
    devs = get_role_devices(config, role)
    if not devs:
        raise DeviceConfigError(f'no devices for role: {role}')
    for d in devs:
        if d['default_for_role']:
            return d
    return devs[0]


# ============================================================
# 매칭 헬퍼
# ============================================================

def _is_placeholder(v: str) -> bool:
    v = v.strip()
    return (not v) or ('REPLACE_ME' in v)

def _has_complete_match(match: dict | None) -> bool:
    match = match or {}
    method = str(match.get('method', '')).strip().lower()
    if method == 'serial':
        return not any(_is_placeholder(str(match.get(k, ''))) for k in ('vendor_id', 'product_id', 'serial_short'))
    if method == 'id_path':
        return not _is_placeholder(str(match.get('id_path', '')))
    return False

def _attached_matches(attached: dict[str, str], config_dev: dict[str, Any]) -> bool:
    match = config_dev.get('match') or {}
    method = str(match.get('method', '')).strip().lower()
    if not _has_complete_match(match):
        return False
    if method == 'serial':
        return (
            attached.get('vendor_id','').strip()    == str(match.get('vendor_id','')).strip()
            and attached.get('product_id','').strip() == str(match.get('product_id','')).strip()
            and attached.get('serial_short','').strip() == str(match.get('serial_short','')).strip()
        )
    if method == 'id_path':
        return attached.get('id_path','').strip() == str(match.get('id_path','')).strip()
    return False

def _find_attached_for(config_dev: dict[str, Any]) -> dict[str, str] | None:
    if not _has_complete_match(config_dev.get('match')):
        return None
    for a in list_serial_devices():
        if _attached_matches(a, config_dev):
            return a
    return None


# ============================================================
# 장치 경로 해석
# ============================================================

def resolve_device_selection(
    config: dict[str, Any], role: str,
    requested_name: str = '', requested_path: str = '',
) -> dict[str, Any]:
    requested_name, requested_path = requested_name.strip(), requested_path.strip()

    if requested_path:
        return {'role': role, 'source': 'path', 'name': requested_name,
                'path': requested_path, 'alias_path': requested_path,
                'path_source': 'path', 'matched_device': None, 'device': None}

    device = get_device_by_name(config, requested_name) if requested_name else get_default_device(config, role)
    source = 'name' if requested_name else 'default'

    if requested_name and device['role'] != role:
        raise DeviceConfigError(f'{requested_name} is role={device["role"]}, expected {role}')

    alias_path = get_device_path(device)
    path, path_source, matched = alias_path, 'symlink', None

    if not Path(alias_path).exists():
        matched = _find_attached_for(device)
        path = matched['device'] if matched else alias_path
        path_source = 'live_match' if matched else 'missing_symlink'

    return {'role': role, 'source': source, 'name': device['name'],
            'path': path, 'alias_path': alias_path, 'path_source': path_source,
            'matched_device': matched, 'device': device}


# ============================================================
# udev rule 생성 / 설치
# ============================================================

def build_udev_rule(device: dict[str, Any]) -> str:
    match = device.get('match') or {}
    method = str(match.get('method', '')).strip().lower()
    if not method:
        raise DeviceConfigError(f'missing match.method for {device["name"]}')

    if method == 'serial':
        vid, pid, ser = str(match.get('vendor_id','')).strip(), str(match.get('product_id','')).strip(), str(match.get('serial_short','')).strip()
        if _is_placeholder(vid) or _is_placeholder(pid) or _is_placeholder(ser):
            raise DeviceConfigError(f'incomplete serial match for {device["name"]}')
        selector = f'ATTRS{{idVendor}}=="{vid}", ATTRS{{idProduct}}=="{pid}", ATTRS{{serial}}=="{ser}"'
    elif method == 'id_path':
        id_path = str(match.get('id_path', '')).strip()
        if _is_placeholder(id_path):
            raise DeviceConfigError(f'incomplete id_path for {device["name"]}')
        selector = f'ENV{{ID_PATH}}=="{id_path}"'
    else:
        raise DeviceConfigError(f'unsupported method for {device["name"]}: {method}')

    symlinks = ', '.join(f'SYMLINK+="{s}"' for s in [device['symlink']] + device.get('extra_symlinks', []))
    return f'SUBSYSTEM=="tty", {selector}, {symlinks}, MODE="0666"'

def render_udev_rules(config: dict[str, Any]) -> tuple[list[str], list[str]]:
    rules, warnings = [], []
    for device in config['devices']:
        try:
            rules.append(build_udev_rule(device))
        except DeviceConfigError as e:
            warnings.append(str(e))
    return rules, warnings

def install_udev_rules(config: dict[str, Any], dest: str) -> tuple[list[str], list[str]]:
    rules, warnings = render_udev_rules(config)
    if not rules:
        raise DeviceConfigError('no valid udev rules to install')

    with tempfile.NamedTemporaryFile('w', encoding='utf-8', delete=False, suffix='.rules') as tmp:
        tmp.write('\n'.join(rules) + '\n')
        tmp_path = tmp.name

    for cmd in [['sudo', 'cp', tmp_path, dest],
                ['sudo', 'udevadm', 'control', '--reload-rules'],
                ['sudo', 'udevadm', 'trigger']]:
        r = subprocess.run(cmd, capture_output=True, text=True, check=False)
        if r.returncode != 0:
            raise DeviceConfigError(r.stderr.strip() or f'failed: {cmd}')

    return rules, warnings


# ============================================================
# 장치 목록 조회
# ============================================================

def _run(cmd: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, check=False)

def _get_props(dev: str) -> dict[str, str]:
    out = _run(['udevadm', 'info', '-q', 'property', '-n', dev])
    if out.returncode != 0:
        return {}
    props = {}
    for line in out.stdout.splitlines():
        if '=' in line:
            k, v = line.split('=', 1)
            props[k.strip()] = v.strip()
    return props

def _find_by_id_link(dev: str) -> str:
    by_id = Path('/dev/serial/by-id')
    if not by_id.exists():
        return ''
    for link in sorted(by_id.iterdir()):
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
    devices = []
    for dev in [l.strip() for l in out.stdout.splitlines() if l.strip()]:
        p = _get_props(dev)
        devices.append({
            'device': dev,
            'vendor_id': p.get('ID_VENDOR_ID', ''),
            'product_id': p.get('ID_MODEL_ID', ''),
            'vendor': p.get('ID_VENDOR', ''),
            'model': p.get('ID_MODEL', ''),
            'serial': p.get('ID_SERIAL', ''),
            'vendor_product': f"{p.get('ID_VENDOR_ID','?')}:{p.get('ID_MODEL_ID','?')}",
            'serial_short': p.get('ID_SERIAL_SHORT', ''),
            'id_path': p.get('ID_PATH', ''),
            'by_id': _find_by_id_link(dev),
        })
    return devices


# ============================================================
# autofill
# ============================================================

def _rule_value(props: dict[str, str], field: str) -> str:
    return {'vendor_id': props.get('vendor_id',''), 'product_id': props.get('product_id',''),
            'vendor': props.get('vendor',''), 'model': props.get('model',''),
            'serial': props.get('serial',''), 'serial_short': props.get('serial_short',''),
            'id_path': props.get('id_path','')}.get(field, '')

def _matches_detection_rule(device: dict[str, str], rule: dict[str, Any]) -> bool:
    for key, expected in (rule.get('match') or {}).items():
        actual = _rule_value(device, key[:-6] if key.endswith('_regex') else key)
        if key.endswith('_regex'):
            if not re.search(str(expected), actual):
                return False
        elif str(actual).strip() != str(expected).strip():
            return False
    return True

def _name_sort_key(name: str) -> tuple:
    m = re.match(r'^([a-zA-Z_]+?)(\d+)$', name)
    return (m.group(1), int(m.group(2)), name) if m else (name, 0, name)

def _build_match_block(attached: dict[str, str]) -> dict[str, str]:
    if attached.get('serial_short'):
        return {'method': 'serial', 'vendor_id': attached.get('vendor_id',''),
                'product_id': attached.get('product_id',''), 'serial_short': attached.get('serial_short','')}
    return {'method': 'id_path', 'id_path': attached.get('id_path','')}

def autofill_device_matches(config: dict[str, Any]) -> tuple[list[str], list[str]]:
    attached = list_serial_devices()
    rules = config.get('detection_rules', [])
    if not rules:
        raise DeviceConfigError('no detection_rules configured')

    matched_by_role: dict[str, list] = defaultdict(list)
    warnings, changes = [], []

    for a in attached:
        rule = next((r for r in rules if _matches_detection_rule(a, r)), None)
        if rule is None:
            warnings.append(f"unmatched: {a['device']} ({a['vendor_product']} {a['model']})")
            continue
        matched_by_role[rule['role']].append(a)

    for role, attached_devs in matched_by_role.items():
        cfg_devs = sorted(get_role_devices(config, role), key=lambda d: _name_sort_key(d['name']))
        attached_devs = sorted(attached_devs, key=lambda d: (d.get('serial_short',''), d.get('id_path',''), d.get('device','')))

        assigned, remaining_cfg, remaining_att = [], [], list(attached_devs)
        for cd in cfg_devs:
            m = next((a for a in remaining_att if _attached_matches(a, cd)), None)
            if m is None:
                remaining_cfg.append(cd)
            else:
                assigned.append((cd, m))
                remaining_att.remove(m)

        if len(attached_devs) > len(cfg_devs):
            warnings.append(f'role={role}: {len(attached_devs)} attached, {len(cfg_devs)} config slots')

        for i, a in enumerate(remaining_att[:len(remaining_cfg)]):
            assigned.append((remaining_cfg[i], a))

        for cd, a in assigned:
            block = _build_match_block(a)
            if cd.get('match') != block:
                cd['match'] = block
                changes.append(f"{cd['name']} -> {a['device']} ({a.get('model','')} {a.get('serial_short','')})")

    return changes, warnings


# ============================================================
# CLI 출력
# ============================================================

def _print_help_menu() -> None:
    print('='*60)
    print('  HENES 포트 고정 관리 도구')
    print('='*60)
    print('  list      : 연결된 장치 목록 표시')
    print('  show      : 설정된 심볼릭 링크 표시')
    print('  render    : udev rule 미리보기')
    print('  autofill  : 연결 장치 자동 감지 (--write 로 저장)')
    print('  install   : udev rule 설치')
    print('  resolve   : 역할별 실제 경로 확인 (--role gps)')
    print('='*60)

def _print_device_list() -> int:
    devices = list_serial_devices()
    if not devices:
        print('연결된 /dev/ttyUSB* 또는 /dev/ttyACM* 장치가 없습니다.')
        return 1
    print(f"{'No':>2} | {'Device':<12} | {'Vendor:Product':<14} | {'Serial':<20} | {'ID_PATH':<32} | by-id")
    print('-' * 110)
    for i, d in enumerate(devices, 1):
        print(f"{i:>2} | {d['device']:<12} | {d['vendor_product']:<14} | "
              f"{d['serial_short'] or 'N/A':<20} | {d['id_path'] or 'N/A':<32} | {d['by_id'] or 'N/A'}")
    return 0

def _print_config(config: dict[str, Any]) -> int:
    grouped: dict[str, list] = defaultdict(list)
    for d in config['devices']:
        grouped[d['role']].append(d)
    print(f"config: {config['path']}")
    for role in sorted(grouped):
        print(f'\n[{role}]')
        for d in grouped[role]:
            extra = f" (+ {', '.join('/dev/'+s for s in d['extra_symlinks'])})" if d.get('extra_symlinks') else ''
            default_text = ' [default]' if d.get('default_for_role') else ''
            print(f"  {d['name']}: {get_device_path(d)}{extra}{default_text}")
    return 0

def _print_rules(config: dict[str, Any]) -> int:
    rules, warnings = render_udev_rules(config)
    for r in rules:
        print(r)
    if warnings:
        print('\n# warnings')
        for w in warnings:
            print(f'# {w}')
    return 0 if rules else 1

def _print_resolution(config: dict[str, Any], role: str, name: str, path: str) -> int:
    sel = resolve_device_selection(config, role, requested_name=name, requested_path=path)
    print(f"role={sel['role']}")
    print(f"source={sel['source']}")
    if sel['name']:
        print(f"name={sel['name']}")
    print(f"path_source={sel.get('path_source','path')}")
    if sel.get('alias_path'):
        print(f"alias_path={sel['alias_path']}")
    print(f"path={sel['path']}")
    return 0


# ============================================================
# CLI 파서 / main
# ============================================================

def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='HENES serial device config and udev rule tool')
    parser.add_argument('--config', default=default_config_path())
    sub = parser.add_subparsers(dest='command')

    sub.add_parser('show',   help='설정된 장치 및 심볼릭 링크 표시')
    sub.add_parser('list',   help='현재 연결된 ttyUSB/ttyACM 장치 목록')
    sub.add_parser('render', help='udev rule 미리보기')

    install_p = sub.add_parser('install', help='udev rule 설치')
    install_p.add_argument('--dest', default='/etc/udev/rules.d/99-henes-serial.rules')

    autofill_p = sub.add_parser('autofill', help='연결 장치 자동 감지 및 YAML 갱신')
    autofill_p.add_argument('--write', action='store_true', help='감지 결과를 YAML 파일에 저장')

    resolve_p = sub.add_parser('resolve', help='역할/이름으로 실제 /dev 경로 확인')
    resolve_p.add_argument('--role', required=True, choices=['gps', 'imu', 'arduino', 'lidar'])
    resolve_p.add_argument('--name', default='')
    resolve_p.add_argument('--path', default='')

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    command = args.command or 'menu'

    if command == 'list':
        return _print_device_list()

    if command == 'menu':
        _print_help_menu()
        print()
        _print_device_list()
        return 0

    config = load_config(args.config)

    if command == 'show':
        return _print_config(config)
    if command == 'render':
        return _print_rules(config)
    if command == 'resolve':
        return _print_resolution(config, args.role, args.name, args.path)
    if command == 'install':
        rules, warnings = install_udev_rules(config, args.dest)
        print(f'설치 완료: {len(rules)}개 rule → {args.dest}')
        for w in warnings:
            print(f'  경고: {w}')
        return 0
    if command == 'autofill':
        changes, warnings = autofill_device_matches(config)
        if changes:
            print('감지된 장치:')
            for c in changes:
                print(f'  - {c}')
        else:
            print('변경 없음')
        for w in warnings:
            print(f'  경고: {w}')
        if args.write:
            save_config(config, args.config)
            print(f'저장 완료: {args.config}')
        return 0

    parser.print_help()
    return 1


# ============================================================
# Launch 파일 호환 alias / 추가 헬퍼
# ============================================================

def format_device_resolution(selection: dict, label: str) -> str:
    if selection['source'] == 'path':
        return f'{label} resolved from explicit path -> {selection["path"]}'
    message = (f'{label} resolved from YAML '
               f'({selection["source"]}: {selection["name"]}) -> {selection["path"]}')
    path_source = selection.get('path_source', 'path')
    if path_source == 'live_match':
        message += f' [live-match fallback, alias missing: {selection.get("alias_path", "")}]'
    elif path_source == 'missing_symlink':
        message += f' [alias missing: {selection.get("alias_path", "")}]'
    return message

# 이름 alias (launch 파일에서 사용하는 이름)
default_device_config_path = default_config_path
load_device_config = load_config
save_device_config = save_config


if __name__ == '__main__':
    try:
        sys.exit(main())
    except DeviceConfigError as e:
        print(f'오류: {e}', file=sys.stderr)
        sys.exit(1)
