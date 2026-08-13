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


def test_world_loads_robot_before_simulation_starts():
    root = ET.parse(PACKAGE / 'worlds/docking_demo.sdf').getroot()
    include = root.find(".//include[name='waffle_pi']")
    render_engine = root.findtext('.//plugin/render_engine')

    assert include is not None
    assert include.findtext('uri') == 'model://turtlebot3_waffle_pi'
    assert include.findtext('pose') == '0.0 0.0 0.01 0 0 0'
    assert render_engine == 'ogre'


def test_demo_map_is_a_free_six_by_four_meter_area():
    metadata = yaml.safe_load((PACKAGE / 'maps/demo_map.yaml').read_text())
    header = (PACKAGE / 'maps/demo_map.pgm').read_text().splitlines()[:3]

    assert metadata['resolution'] == 0.05
    assert metadata['origin'] == [-3.0, -2.0, 0.0]
    assert header == ['P2', '120 80', '255']


def test_single_bridge_maps_camera_image_and_info():
    mappings = yaml.safe_load(
        (PACKAGE / 'config/turtlebot3_bridge.yaml').read_text()
    )
    by_topic = {mapping['ros_topic_name']: mapping for mapping in mappings}

    assert by_topic['camera/image_raw']['gz_type_name'] == 'gz.msgs.Image'
    assert by_topic['camera/camera_info']['gz_type_name'] == 'gz.msgs.CameraInfo'
    assert all(
        mapping['direction'] in {'GZ_TO_ROS', 'ROS_TO_GZ'}
        for mapping in mappings
    )


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
    assert "executable='create'" not in source
    assert 'ros_gz_image' not in source


def test_python_bridges_treat_context_shutdown_as_clean_exit():
    for path in (
        PACKAGE / 'demo2_apriltag_docking/docking_task_bridge.py',
        PACKAGE / 'demo2_apriltag_docking/tag_pose_bridge.py',
    ):
        source = path.read_text(encoding='utf-8')
        assert 'from rclpy.executors import ExternalShutdownException' in source
        assert 'except (KeyboardInterrupt, ExternalShutdownException):' in source


def test_task_bridge_selector_preserves_python_default():
    source = (PACKAGE / 'launch/demo.launch.py').read_text(encoding='utf-8')
    tree = ast.parse(source)

    assignments = {
        target.id: node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        for target in node.targets
        if isinstance(target, ast.Name)
    }
    python_task = assignments['python_task_bridge']
    cpp_task = assignments['cpp_task_bridge']

    assert isinstance(python_task, ast.Call)
    assert isinstance(cpp_task, ast.Call)
    assert python_task.func.id == cpp_task.func.id == 'Node'

    def keywords(call):
        return {keyword.arg: keyword.value for keyword in call.keywords}

    python_keywords = keywords(python_task)
    cpp_keywords = keywords(cpp_task)
    assert python_keywords['package'].value == 'demo2_apriltag_docking'
    assert python_keywords['executable'].value == 'docking_task_bridge'
    assert cpp_keywords['package'].value == 'demo2_apriltag_docking_cpp'
    assert cpp_keywords['executable'].value == 'docking_task_bridge_cpp'
    assert python_keywords['name'].value == 'docking_task_bridge'
    assert cpp_keywords['name'].value == 'docking_task_bridge'
    assert ast.dump(python_keywords['parameters']) == ast.dump(
        cpp_keywords['parameters']
    )

    for call, implementation in ((python_task, 'python'), (cpp_task, 'cpp')):
        condition = keywords(call)['condition']
        assert condition.func.id == 'IfCondition'
        equals = condition.args[0]
        assert equals.func.id == 'EqualsSubstitution'
        assert equals.args[0].id == 'task_bridge_implementation'
        assert equals.args[1].value == implementation

    declarations = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'DeclareLaunchArgument'
        and node.args
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value == 'task_bridge_implementation'
    ]
    assert len(declarations) == 1
    declaration = keywords(declarations[0])
    assert declaration['default_value'].value == 'python'
    assert [item.value for item in declaration['choices'].elts] == [
        'python', 'cpp',
    ]


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
    arguments_dict = launch_arguments.func.value
    server_args = next(
        value
        for key, value in zip(arguments_dict.keys, arguments_dict.values)
        if isinstance(key, ast.Constant) and key.value == 'gz_args'
    )

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
