#!/usr/bin/env python3

import argparse
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


def parse_args():
    parser = argparse.ArgumentParser(description='生成项目侧 Piper 描述，支持占位模型或 AgileX 官方 URDF 适配。')
    parser.add_argument('--base-xacro', default='', help='可选：移动底盘 xacro。为空时只生成 Piper 挂载描述。')
    parser.add_argument('--placeholder-xacro', default='', help='项目侧占位 Piper xacro。')
    parser.add_argument('--enable-nav-rgbd-camera', default='false')
    parser.add_argument('--enable-piper-arm', default='true')
    parser.add_argument('--arm-model', choices=['placeholder', 'official'], default='official')
    parser.add_argument('--official-description-package', default='piper_description')
    parser.add_argument('--official-description-xacro', default='urdf/piper_description.xacro')
    parser.add_argument('--mount-xyz', default='0.16 0.0 0.22')
    parser.add_argument('--mount-rpy', default='0 0 0')
    parser.add_argument('--base-offset-xyz', default='0 0 0.04')
    parser.add_argument('--base-offset-rpy', default='0 0 0')
    parser.add_argument('--tcp-parent-link', default='piper_link6')
    parser.add_argument('--tcp-xyz', default='0 0 0')
    parser.add_argument('--tcp-rpy', default='0 0 0')
    parser.add_argument('--camera-xyz', default='0.04 0.0 0.04')
    parser.add_argument('--camera-rpy', default='0 0 0')
    parser.add_argument('--enable-piper-gazebo-camera', default='false')
    parser.add_argument('--piper-gazebo-camera-width', default='640')
    parser.add_argument('--piper-gazebo-camera-height', default='480')
    parser.add_argument('--piper-gazebo-camera-update-rate', default='15')
    parser.add_argument('--piper-gazebo-camera-fov', default='1.20')
    parser.add_argument('--piper-gazebo-camera-min-depth', default='0.10')
    parser.add_argument('--piper-gazebo-camera-max-depth', default='3.00')
    return parser.parse_args()


def to_bool(value):
    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


def require_xyz(name, value):
    parts = str(value).replace(',', ' ').split()
    if len(parts) != 3:
        raise ValueError(f'{name} 必须包含 3 个数，当前值为: {value}')
    return ' '.join(parts)


def run_xacro(path, mappings):
    cmd = ['xacro', str(path)]
    cmd.extend(f'{key}:={value}' for key, value in mappings.items())
    result = subprocess.run(cmd, check=False, text=True, capture_output=True)
    if result.returncode != 0:
        message = result.stderr.strip() or result.stdout.strip()
        raise RuntimeError(f'xacro 渲染失败: {path}\n{message}')
    return ET.fromstring(result.stdout)


def add_origin(element, xyz='0 0 0', rpy='0 0 0'):
    ET.SubElement(element, 'origin', {'xyz': xyz, 'rpy': rpy})


def add_box_link(root, name, size, color, mass='0.10'):
    link = ET.SubElement(root, 'link', {'name': name})
    visual = ET.SubElement(link, 'visual')
    add_origin(visual)
    geometry = ET.SubElement(visual, 'geometry')
    ET.SubElement(geometry, 'box', {'size': size})
    material = ET.SubElement(visual, 'material', {'name': f'{name}_material'})
    ET.SubElement(material, 'color', {'rgba': color})

    collision = ET.SubElement(link, 'collision')
    add_origin(collision)
    collision_geometry = ET.SubElement(collision, 'geometry')
    ET.SubElement(collision_geometry, 'box', {'size': size})

    # Gazebo 需要惯量；这里给挂载板/相机盒一个保守的轻量惯量，真实硬件后续按实物修正。
    x, y, z = [float(item) for item in size.split()]
    m = float(mass)
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass', {'value': str(m)})
    add_origin(inertial)
    ET.SubElement(
        inertial,
        'inertia',
        {
            'ixx': str(m * (y * y + z * z) / 12.0),
            'ixy': '0',
            'ixz': '0',
            'iyy': str(m * (x * x + z * z) / 12.0),
            'iyz': '0',
            'izz': str(m * (x * x + y * y) / 12.0),
        },
    )


def add_empty_link(root, name):
    if root.find(f"./link[@name='{name}']") is None:
        ET.SubElement(root, 'link', {'name': name})


def add_fixed_joint(root, name, parent, child, xyz='0 0 0', rpy='0 0 0'):
    joint = ET.SubElement(root, 'joint', {'name': name, 'type': 'fixed'})
    ET.SubElement(joint, 'parent', {'link': parent})
    ET.SubElement(joint, 'child', {'link': child})
    add_origin(joint, xyz, rpy)


def add_text(parent, tag, text):
    element = ET.SubElement(parent, tag)
    element.text = str(text)
    return element


def add_mount(root, parent_frame, args):
    if root.find("./link[@name='piper_mount_link']") is None:
        add_box_link(root, 'piper_mount_link', '0.20 0.18 0.04', '0.25 0.27 0.30 1.0', mass='0.45')
    if root.find("./joint[@name='piper_mount_joint']") is None:
        add_fixed_joint(
            root,
            'piper_mount_joint',
            parent_frame,
            'piper_mount_link',
            require_xyz('mount_xyz', args.mount_xyz),
            require_xyz('mount_rpy', args.mount_rpy),
        )


def add_tcp_and_camera(root, tcp_parent, args):
    add_empty_link(root, 'piper_tcp')
    if root.find("./joint[@name='piper_tcp_joint']") is None:
        add_fixed_joint(
            root,
            'piper_tcp_joint',
            tcp_parent,
            'piper_tcp',
            require_xyz('tcp_xyz', args.tcp_xyz),
            require_xyz('tcp_rpy', args.tcp_rpy),
        )

    if root.find("./link[@name='piper_arm_camera_link']") is None:
        add_box_link(root, 'piper_arm_camera_link', '0.055 0.075 0.035', '0.01 0.01 0.01 1.0', mass='0.10')
    if root.find("./joint[@name='piper_arm_camera_joint']") is None:
        add_fixed_joint(
            root,
            'piper_arm_camera_joint',
            'piper_tcp',
            'piper_arm_camera_link',
            require_xyz('camera_xyz', args.camera_xyz),
            require_xyz('camera_rpy', args.camera_rpy),
        )

    add_empty_link(root, 'piper_arm_camera_optical_frame')
    if root.find("./joint[@name='piper_arm_camera_optical_joint']") is None:
        # ROS 相机光学坐标系：x 向右，y 向下，z 向前。
        add_fixed_joint(
            root,
            'piper_arm_camera_optical_joint',
            'piper_arm_camera_link',
            'piper_arm_camera_optical_frame',
            '0 0 0',
            '-1.5707963267948966 0 -1.5707963267948966',
        )

    if to_bool(args.enable_piper_gazebo_camera):
        add_gazebo_arm_camera(root, args)


def add_gazebo_arm_camera(root, args):
    if root.find("./gazebo[@reference='piper_arm_camera_link']/sensor[@name='piper_arm_camera']") is not None:
        return

    gazebo = ET.SubElement(root, 'gazebo', {'reference': 'piper_arm_camera_link'})
    sensor = ET.SubElement(gazebo, 'sensor', {'name': 'piper_arm_camera', 'type': 'depth'})
    add_text(sensor, 'always_on', 'true')
    add_text(sensor, 'update_rate', args.piper_gazebo_camera_update_rate)
    add_text(sensor, 'visualize', 'true')

    camera = ET.SubElement(sensor, 'camera')
    add_text(camera, 'horizontal_fov', args.piper_gazebo_camera_fov)
    image = ET.SubElement(camera, 'image')
    add_text(image, 'width', args.piper_gazebo_camera_width)
    add_text(image, 'height', args.piper_gazebo_camera_height)
    add_text(image, 'format', 'B8G8R8')
    ET.SubElement(camera, 'depth_camera')
    clip = ET.SubElement(camera, 'clip')
    add_text(clip, 'near', args.piper_gazebo_camera_min_depth)
    add_text(clip, 'far', args.piper_gazebo_camera_max_depth)
    noise = ET.SubElement(camera, 'noise')
    add_text(noise, 'type', 'gaussian')
    add_text(noise, 'mean', '0.0')
    add_text(noise, 'stddev', '0.003')

    plugin = ET.SubElement(sensor, 'plugin', {'name': 'piper_arm_camera_controller', 'filename': 'libgazebo_ros_camera.so'})
    ros = ET.SubElement(plugin, 'ros')
    # Gazebo 腕部相机只发布到 /piper/arm_camera/*，不能复用 /nav_camera。
    add_text(ros, 'namespace', '/piper/arm_camera')
    for remap in (
        'wrist_rgbd/image_raw:=color/image_raw',
        'wrist_rgbd/depth/image_raw:=depth/image_raw',
        'wrist_rgbd/camera_info:=color/camera_info',
        'wrist_rgbd/depth/camera_info:=depth/camera_info',
        'wrist_rgbd/points:=depth/points',
    ):
        add_text(ros, 'remapping', remap)
    add_text(plugin, 'camera_name', 'wrist_rgbd')
    add_text(plugin, 'frame_name', 'piper_arm_camera_optical_frame')
    add_text(plugin, 'min_depth', args.piper_gazebo_camera_min_depth)
    add_text(plugin, 'max_depth', args.piper_gazebo_camera_max_depth)


def official_path(package_name, relative_path):
    try:
        share_dir = Path(get_package_share_directory(package_name))
    except PackageNotFoundError as exc:
        raise RuntimeError(
            f'未找到 AgileX 官方描述包 {package_name}。请先运行 scripts/setup_piper_open_class.sh 并 source install/setup.bash。'
        ) from exc
    path = share_dir / relative_path
    if not path.exists():
        raise RuntimeError(f'官方 Piper xacro 不存在: {path}')
    return path


def build_name_maps(official_root):
    link_names = [element.attrib['name'] for element in official_root.findall('link') if 'name' in element.attrib]
    joint_names = [element.attrib['name'] for element in official_root.findall('joint') if 'name' in element.attrib]

    link_map = {}
    for name in link_names:
        if name == 'world':
            continue
        if name == 'base_link':
            link_map[name] = 'piper_base_link'
        elif re.fullmatch(r'link\d+', name):
            link_map[name] = f'piper_{name}'
        elif name.startswith('piper_'):
            link_map[name] = name
        else:
            link_map[name] = f'piper_{name}'

    joint_map = {}
    for name in joint_names:
        if name == 'fixed_base_joint':
            continue
        if re.fullmatch(r'joint\d+', name):
            joint_map[name] = f'piper_{name}'
        elif name.startswith('piper_'):
            joint_map[name] = name
        else:
            joint_map[name] = f'piper_{name}'

    return link_map, joint_map


def update_references(root, link_map, joint_map):
    for element in root.iter():
        if element.tag == 'link' and 'name' in element.attrib:
            element.attrib['name'] = link_map.get(element.attrib['name'], element.attrib['name'])
        elif element.tag == 'joint' and 'name' in element.attrib:
            element.attrib['name'] = joint_map.get(element.attrib['name'], element.attrib['name'])

        if element.tag in {'parent', 'child'} and 'link' in element.attrib:
            element.attrib['link'] = link_map.get(element.attrib['link'], element.attrib['link'])
        if element.tag == 'mimic' and 'joint' in element.attrib:
            element.attrib['joint'] = joint_map.get(element.attrib['joint'], element.attrib['joint'])

        for key in ('link', 'link1', 'link2', 'reference'):
            if key in element.attrib:
                element.attrib[key] = link_map.get(element.attrib[key], element.attrib[key])
        for key in ('joint', 'joint1', 'joint2'):
            if key in element.attrib:
                element.attrib[key] = joint_map.get(element.attrib[key], element.attrib[key])


def append_official_arm(root, args):
    path = official_path(args.official_description_package, args.official_description_xacro)
    official_root = run_xacro(path, {})
    link_map, joint_map = build_name_maps(official_root)

    for element in list(official_root):
        if element.tag == 'link' and element.attrib.get('name') == 'world':
            official_root.remove(element)
        elif element.tag == 'joint' and element.attrib.get('name') == 'fixed_base_joint':
            official_root.remove(element)

    update_references(official_root, link_map, joint_map)
    add_fixed_joint(
        root,
        'piper_base_mount_joint',
        'piper_mount_link',
        'piper_base_link',
        require_xyz('base_offset_xyz', args.base_offset_xyz),
        require_xyz('base_offset_rpy', args.base_offset_rpy),
    )

    for element in list(official_root):
        root.append(element)

    tcp_parent = args.tcp_parent_link
    if tcp_parent in link_map:
        tcp_parent = link_map[tcp_parent]
    elif re.fullmatch(r'link\d+', tcp_parent):
        tcp_parent = f'piper_{tcp_parent}'
    add_tcp_and_camera(root, tcp_parent, args)


def build_standalone_official(args):
    root = ET.Element('robot', {'name': 'slam_nav_piper_official'})
    add_empty_link(root, 'base_link')
    add_mount(root, 'base_link', args)
    append_official_arm(root, args)
    return root


def build_from_base(args):
    enable_piper = to_bool(args.enable_piper_arm)
    base_root = run_xacro(
        args.base_xacro,
        {
            'enable_nav_rgbd_camera': args.enable_nav_rgbd_camera,
            # 官方模式下先关闭内置占位链，再把官方链路追加到同一棵 robot_description。
            'enable_piper_arm': 'true' if enable_piper and args.arm_model == 'placeholder' else 'false',
        },
    )
    if enable_piper and args.arm_model == 'official':
        add_mount(base_root, 'base_link', args)
        append_official_arm(base_root, args)
    return base_root


def main():
    args = parse_args()
    try:
        if args.base_xacro:
            root = build_from_base(args)
        elif args.arm_model == 'official':
            root = build_standalone_official(args)
        else:
            if not args.placeholder_xacro:
                raise RuntimeError('placeholder 模式需要提供 --placeholder-xacro。')
            root = run_xacro(
                args.placeholder_xacro,
                {
                    'mount_xyz': args.mount_xyz,
                    'mount_rpy': args.mount_rpy,
                    'camera_xyz': args.camera_xyz,
                    'camera_rpy': args.camera_rpy,
                },
            )
        ET.indent(root, space='  ')
        print(ET.tostring(root, encoding='unicode'))
    except Exception as exc:
        print(f'[Piper 描述生成失败] {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
