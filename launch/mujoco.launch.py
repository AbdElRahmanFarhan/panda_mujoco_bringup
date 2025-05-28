import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    panda_mujoco_path = os.path.join(
        get_package_share_directory('panda_mujoco_bringup'))

    xacro_file = os.path.join(panda_mujoco_path,
                              'config',
                              'panda.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    panda_moveit_config_path = os.path.join(
        get_package_share_directory('moveit_resources_panda_moveit_config'))

    controller_config_file = os.path.join(panda_moveit_config_path, 'config', 'ros2_controllers.yaml')

    mujoco_models = os.path.join('/home', 'ros2_ws', 'src','mujoco_menagerie')
    panda_mujoco_model = os.path.join(panda_mujoco_path, 'mujoco_model', 'panda.xml')

    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_config_file,
            {'mujoco_model_path': panda_mujoco_model}
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_panda_arm_controller= ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'panda_arm_controller'],
        output='screen'
    )

    load_panda_hand_controller= ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'panda_hand_controller'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco_ros2_control,
                on_start=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_panda_hand_controller, load_panda_arm_controller],
            )
        ),
        node_mujoco_ros2_control,
        node_robot_state_publisher
    ])
