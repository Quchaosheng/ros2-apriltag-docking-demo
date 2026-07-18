import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


def make_status(node_name, level, message, values):
    status = DiagnosticStatus()
    status.name = node_name
    status.hardware_id = 'demo2_apriltag_docking'
    status.level = level
    status.message = message
    status.values = [
        KeyValue(key=str(key), value=str(value))
        for key, value in values.items()
    ]
    return status


def shutdown_if_running():
    if rclpy.ok():
        rclpy.shutdown()
