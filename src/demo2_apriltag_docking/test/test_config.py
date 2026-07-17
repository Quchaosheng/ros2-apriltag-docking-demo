from pathlib import Path

import yaml

from demo2_apriltag_docking.tag_policy import load_dock_specs


CONFIG = Path(__file__).parents[1] / 'config'


def load(name):
    with (CONFIG / name).open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def test_dock_mapping_matches_demo_tag():
    mapping = load('docks.yaml')
    specs = load_dock_specs(mapping)

    assert specs[0].dock_id == 'demo_charge_dock'
    assert specs[0].dock_type == 'charging_dock'
    assert specs[0].tag_frame == 'tag36h11:0'


def test_apriltag_detector_uses_expected_family_and_size():
    parameters = load('apriltag.yaml')['apriltag']['ros__parameters']

    assert parameters['family'] == '36h11'
    assert parameters['size'] == 0.16
    assert parameters['max_hamming'] == 0
    assert parameters['pose_estimation_method'] == 'pnp'
    assert 'tag.ids' not in parameters


def test_nav2_docking_uses_external_pose_and_bounded_retries():
    parameters = load('nav2_docking.yaml')['docking_server']['ros__parameters']
    plugin = parameters['charging_dock']

    assert parameters['max_retries'] == 2
    assert parameters['dock_plugins'] == ['charging_dock']
    assert plugin['plugin'] == 'opennav_docking::SimpleChargingDock'
    assert plugin['use_external_detection_pose'] is True
    assert plugin['external_detection_timeout'] == 0.8
    assert plugin['use_battery_status'] is False


def test_dock_database_uses_same_dock_id_and_type():
    dock = load('dock_database.yaml')['docks']['demo_charge_dock']

    assert dock['type'] == 'charging_dock'
    assert dock['frame'] == 'odom'
    assert dock['pose'] == [2.0, 0.0, 0.0]
