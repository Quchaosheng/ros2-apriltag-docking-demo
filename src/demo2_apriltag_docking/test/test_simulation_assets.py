from pathlib import Path
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
    assert "['-r -s -v2 \"', world, '\"']" in source
