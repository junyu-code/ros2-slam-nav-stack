#!/usr/bin/env python3

import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


KEY_FRAME_HINTS = ('tcp', 'flange', 'link6', 'link7', 'eef', 'end_effector', 'gripper')


def find_description_files(share_dir):
    urdf_dir = Path(share_dir) / 'urdf'
    if not urdf_dir.exists():
        return []
    return sorted(
        [
            path
            for path in urdf_dir.rglob('*')
            if path.suffix in {'.urdf', '.xacro'} or path.name.endswith('.urdf.xacro')
        ]
    )


def render_description(path):
    if path.suffix == '.xacro' or path.name.endswith('.urdf.xacro'):
        result = subprocess.run(
            ['xacro', str(path)],
            check=False,
            text=True,
            capture_output=True,
        )
        if result.returncode != 0:
            return None, result.stderr.strip() or result.stdout.strip()
        return result.stdout, ''

    return path.read_text(encoding='utf-8'), ''


def parse_frames(xml_text):
    root = ET.fromstring(xml_text)
    links = {element.attrib['name'] for element in root.findall('link') if 'name' in element.attrib}
    child_links = set()
    joints = []

    for joint in root.findall('joint'):
        parent = joint.find('parent')
        child = joint.find('child')
        if parent is None or child is None:
            continue
        parent_link = parent.attrib.get('link')
        child_link = child.attrib.get('link')
        if parent_link and child_link:
            joints.append((joint.attrib.get('name', ''), parent_link, child_link))
            child_links.add(child_link)

    roots = sorted(links - child_links)
    key_frames = sorted(
        [
            link
            for link in links
            if any(hint in link.lower() for hint in KEY_FRAME_HINTS)
        ]
    )
    return links, joints, roots, key_frames


def audit_description_file(path):
    xml_text, error = render_description(path)
    if error:
        return {
            'path': path,
            'ok': False,
            'error': error,
            'roots': [],
            'key_frames': [],
            'link_count': 0,
            'joint_count': 0,
        }

    try:
        links, joints, roots, key_frames = parse_frames(xml_text)
    except Exception as exc:
        return {
            'path': path,
            'ok': False,
            'error': str(exc),
            'roots': [],
            'key_frames': [],
            'link_count': 0,
            'joint_count': 0,
        }

    return {
        'path': path,
        'ok': True,
        'error': '',
        'roots': roots,
        'key_frames': key_frames,
        'link_count': len(links),
        'joint_count': len(joints),
    }


def audit_xml_text(title, xml_text):
    links, joints, roots, key_frames = parse_frames(xml_text)
    print(f'\n[{title}]')
    print(f'  links={len(links)}, joints={len(joints)}')
    print(f'  root frames: {", ".join(roots) or "(none)"}')
    print(f'  key frames: {", ".join(key_frames) or "(none)"}')
    return links, joints, roots, key_frames


def render_project_adapter(args):
    try:
        adapter_share = get_package_share_directory(args.adapter_package)
    except PackageNotFoundError as exc:
        raise RuntimeError(f'未找到项目侧描述包 {args.adapter_package}。') from exc

    builder_path = Path(adapter_share) / 'scripts' / 'piper_description_builder.py'
    if not builder_path.exists():
        raise RuntimeError(f'未找到项目侧 Piper 描述生成器: {builder_path}')

    result = subprocess.run(
        [
            'python3',
            str(builder_path),
            '--arm-model',
            'official',
            '--official-description-package',
            args.description_package,
            '--official-description-xacro',
            args.official_description_xacro,
            '--tcp-parent-link',
            args.official_tcp_parent_frame,
        ],
        check=False,
        text=True,
        capture_output=True,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip())
    return result.stdout


def main():
    parser = argparse.ArgumentParser(description='审计 AgileX 官方 Piper URDF/MoveIt2 frame。')
    parser.add_argument('--description-package', default='piper_description')
    parser.add_argument('--official-description-xacro', default='urdf/piper_description.xacro')
    parser.add_argument('--official-tcp-parent-frame', default='piper_link6')
    parser.add_argument('--adapter-package', default='slam_nav_piper_description')
    parser.add_argument('--expected-root', default='base_link')
    parser.add_argument('--expected-tcp', default='link6')
    parser.add_argument('--check-project-adapter', action='store_true')
    parser.add_argument('--require-expected-frames', action='store_true')
    args = parser.parse_args()

    try:
        share_dir = get_package_share_directory(args.description_package)
    except PackageNotFoundError:
        print(f'未找到官方描述包 {args.description_package}。请先运行 scripts/setup_piper_open_class.sh 并 source install/setup.bash。')
        return 2

    print(f'[官方描述包] {args.description_package}: {share_dir}')
    description_files = find_description_files(share_dir)
    if not description_files:
        print('未找到 urdf/*.urdf 或 urdf/*.xacro 文件。')
        return 2

    all_roots = set()
    all_key_frames = set()
    ok_count = 0

    for path in description_files:
        report = audit_description_file(path)
        print(f'\n[文件] {path}')
        if not report['ok']:
            print(f'  解析失败: {report["error"]}')
            continue
        ok_count += 1
        all_roots.update(report['roots'])
        all_key_frames.update(report['key_frames'])
        print(f'  links={report["link_count"]}, joints={report["joint_count"]}')
        print(f'  root frames: {", ".join(report["roots"]) or "(none)"}')
        print(f'  key frames: {", ".join(report["key_frames"]) or "(none)"}')

    print('\n[汇总]')
    print(f'  可解析描述文件: {ok_count}/{len(description_files)}')
    print(f'  所有 root frames: {", ".join(sorted(all_roots)) or "(none)"}')
    print(f'  所有候选 TCP/末端 frames: {", ".join(sorted(all_key_frames)) or "(none)"}')

    expected_root_ok = args.expected_root in all_roots or args.expected_root in all_key_frames
    expected_tcp_ok = args.expected_tcp in all_key_frames

    if 'base_link' in all_roots:
        print('  注意：官方模型根 frame 包含 base_link，接移动底盘前需要 prefix 或根链接适配。')
    if not expected_root_ok:
        print(f'  注意：未直接找到项目期望根 frame {args.expected_root}。')
    if not expected_tcp_ok:
        print(f'  注意：未直接找到项目期望 TCP frame {args.expected_tcp}。')

    adapter_ok = True
    if args.check_project_adapter:
        try:
            adapter_xml = render_project_adapter(args)
            adapter_links, _, adapter_roots, adapter_key_frames = audit_xml_text('项目侧 piper_* 适配链', adapter_xml)
            adapter_ok = (
                'piper_base_link' in adapter_links
                and 'piper_tcp' in adapter_links
                and 'piper_arm_camera_optical_frame' in adapter_links
            )
            if 'base_link' in adapter_roots:
                print('  项目适配链保留 base_link 作为移动底盘父 frame，这是预期行为。')
            if adapter_ok:
                print('  项目适配链包含 piper_base_link、piper_tcp、piper_arm_camera_optical_frame。')
            else:
                print('  注意：项目适配链缺少 piper_base_link / piper_tcp / piper_arm_camera_optical_frame。')
        except Exception as exc:
            adapter_ok = False
            print(f'\n[项目侧 piper_* 适配链]\n  渲染失败: {exc}')

    if args.require_expected_frames and (not expected_root_ok or not expected_tcp_ok or not adapter_ok):
        return 3
    return 0 if ok_count > 0 else 2


if __name__ == '__main__':
    sys.exit(main())
