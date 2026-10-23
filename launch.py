"""ROBOTIS follower controllers with the settings used for the bundled takes."""
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    sys.path.insert(0, str(Path(__file__).parent))
    from hardware import follower_description
    mock = LaunchConfiguration('mock').perform(context) == 'true'
    actions = []

    def stopped(event, _context):
        if not _context.is_shutdown:
            return [LogInfo(msg='Hardware controller stopped.'), EmitEvent(event=Shutdown())]
        return []

    for side in ('left', 'right'):
        port = LaunchConfiguration(f'{side}_port').perform(context)
        urdf = follower_description(side, port, mock)
        origin = urdf.find(f"joint[@name='{side}_world_fixed']/origin")
        origin.set('xyz', f'0 {0.25 if side == "left" else -0.25} 0')
        description = ParameterValue(ET.tostring(urdf, encoding='unicode'), value_type=str)
        manager = Node(package='controller_manager', executable='ros2_control_node',
                       namespace=side, output='screen', parameters=[{
                           'robot_description': description, 'update_rate': 100,
                           'enforce_command_limits': False,
                           'replay_controller.type': 'joint_trajectory_controller/JointTrajectoryController',
                           'joint_state_broadcaster.type': 'joint_state_broadcaster/JointStateBroadcaster'}])
        state = Node(package='robot_state_publisher', executable='robot_state_publisher',
                     namespace=side, output='screen', parameters=[{'robot_description': description}])
        spawn = Node(package='controller_manager', executable='spawner', namespace=side,
                     arguments=['replay_controller', 'joint_state_broadcaster', '--controller-manager',
                                f'/{side}/controller_manager', '--controller-manager-timeout', '60',
                                '--param-file', str(Path(__file__).with_name('controllers.yaml'))],
                     output='screen')
        actions.extend([RegisterEventHandler(OnProcessExit(target_action=manager, on_exit=stopped)),
                        manager, state, spawn])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false'),
        DeclareLaunchArgument('left_port', default_value='/dev/unused-left'),
        DeclareLaunchArgument('right_port', default_value='/dev/unused-right'),
        OpaqueFunction(function=setup)])
