from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    joystick_node = Node(
        package="joystick_ros2",
        executable="joystick_ros2",
    )

    can_rx_node = Node(
        package="can",
        executable="can_rx_node"
    )

    can_tx_node = Node(
        package="can",
        executable="can_tx_node"
    )

    car_control_node = Node(
        package="car_control",
        executable="car_control_node"
    )

    ld.add_action(joystick_node)
    ld.add_action(can_rx_node)
    ld.add_action(can_tx_node)
    ld.add_action(car_control_node)

    return ld