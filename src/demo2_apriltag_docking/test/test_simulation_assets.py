import ast
from pathlib import Path
import shlex
import struct
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).parents[1]


def test_official_tag_texture_is_valid_png():
    path = PACKAGE / 'models/apriltag_dock/materials/textures/tag36_11_00000.png'
    data = path.read_bytes()

    assert data[:8] == b'\x89PNG\r\n\x1a\n'
    width, height = struct.unpack('>II', data[16:24])
    assert (width, height) == (512, 512)
    assert data[24:26] == bytes([8, 2])


def test_dock_model_references_tag_texture():
    root = ET.parse(PACKAGE / 'models/apriltag_dock/model.sdf').getroot()
    tag_visual = root.find(".//visual[@name='tag_visual']")
    uri = tag_visual.find('./material/pbr/metal/albedo_map')
    size = tag_visual.findtext('./geometry/box/size')

    assert root.find(".//model[@name='apriltag_dock']") is not None
    assert uri is not None
    assert uri.text.endswith('tag36_11_00000.png')
    assert size == '0.002 0.20 0.20'
    assert root.find('.//material/pbr/metal/workflow') is None


def test_world_places_dock_at_database_pose():
    root = ET.parse(PACKAGE / 'worlds/docking_demo.sdf').getroot()
    include = root.find(".//include[name='demo_charge_dock']")

    assert include is not None
    assert include.findtext('pose') == '2.0 0.0 0.0 0 0 0'


def test_demo_map_is_a_free_six_by_four_meter_area():
    metadata = yaml.safe_load((PACKAGE / 'maps/demo_map.yaml').read_text())
    header = (PACKAGE / 'maps/demo_map.pgm').read_text().splitlines()[:3]

    assert metadata['resolution'] == 0.05
    assert metadata['origin'] == [-3.0, -2.0, 0.0]
    assert header == ['P2', '120 80', '255']


def test_launch_file_is_valid_python():
    source = (PACKAGE / 'launch/demo.launch.py').read_text(encoding='utf-8')
    nav2_source = (
        PACKAGE / 'config/turtlebot3_waffle_pi_nav2.yaml'
    ).read_text(encoding='utf-8')

    compile(source, 'demo.launch.py', 'exec')
    for package in (
        'ros_gz_sim',
        'turtlebot3_gazebo',
        'turtlebot3_navigation2',
        'apriltag_ros',
    ):
        assert package in source
    assert 'opennav_docking::SimpleChargingDock' in nav2_source
    assert "package='ros_gz_sim'" in source
    assert "executable='create'" in source


def test_gazebo_server_uses_fixed_seed():
    source = (PACKAGE / 'launch/demo.launch.py').read_text(encoding='utf-8')
    gz_server_assignments = [
        node
        for node in ast.walk(ast.parse(source))
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == 'gz_server'
            for target in node.targets
        )
    ]

    assert len(gz_server_assignments) == 1
    gz_server_call = gz_server_assignments[0].value
    assert isinstance(gz_server_call, ast.Call)
    assert isinstance(gz_server_call.func, ast.Name)
    assert gz_server_call.func.id == 'IncludeLaunchDescription'

    launch_arguments = next(
        keyword.value for keyword in gz_server_call.keywords
        if keyword.arg == 'launch_arguments'
    )
    assert isinstance(launch_arguments, ast.Call)
    assert isinstance(launch_arguments.func, ast.Attribute)
    assert launch_arguments.func.attr == 'items'
    arguments_dict = launch_arguments.func.value
    assert isinstance(arguments_dict, ast.Dict)

    server_args = next(
        value
        for key, value in zip(arguments_dict.keys, arguments_dict.values)
        if isinstance(key, ast.Constant) and key.value == 'gz_args'
    )
    assert isinstance(server_args, ast.List)

    parts = []
    world_references = 0
    for value in server_args.elts:
        if isinstance(value, ast.Constant) and isinstance(value.value, str):
            parts.append(value.value)
        elif isinstance(value, ast.Name) and value.id == 'world':
            parts.append('/tmp/world.sdf')
            world_references += 1
        else:
            raise AssertionError(f'Unexpected gz_server argument: {ast.dump(value)}')

    assert world_references == 1
    assert shlex.split(''.join(parts)) == [
        '-r', '-s', '-v2', '--seed', '42', '/tmp/world.sdf',
    ]
