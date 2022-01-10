# Copyright 2021 Open Robotics (2021)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_ros.actions

import xacro
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # moveit_cpp.yaml is passed by filename for now since it's node specific
    reachy_gazebo = os.path.join(
        get_package_share_directory('reachy_gazebo'),
        'worlds',
        'reachy.world')

    print(reachy_gazebo)

    # simu_params = load_yaml('reachy_gazebo', 'config/simu.yaml')
    simu_params = os.path.join(get_package_share_directory(
        'reachy_gazebo'), 'config', 'simu.yaml')

    print(simu_params)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': reachy_gazebo}.items(),
    )

    reachy_description_path = os.path.join(
        get_package_share_directory('reachy_description'))

    xacro_file = os.path.join(reachy_description_path,
                              'urdf',
                              'reachy.URDF.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, simu_params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'reachy'],
                        output='screen',
                        parameters=[simu_params])

    # load_joint_state_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_start_controller', 'joint_state_controller'],
    #     output='screen'
    # )

    # load_joint_trajectory_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_start_controller',
    #          'joint_trajectory_controller'],
    #     output='screen'
    # )

    reachy_controllers = PathJoinSubstitution(
        [
            FindPackageShare("reachy_gazebo"),
            "config",
            "reachy_gazebo_controllers.yaml",
        ]
    )
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, reachy_controllers, simu_params],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    spawn_jsb_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
        parameters=[simu_params]
    )
    spawn_forward_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["forward_position_controller", "-c", "/controller_manager"],
        output="screen",
        parameters=[simu_params]
    )

    # Static TF
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0',
                                '0.0', '0.0', 'world', 'pedestal'],
                     parameters=[simu_params]
                     )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("reachy_gazebo"),
            "config",
            "reachy_gazebo_controllers.yaml",
        ]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers, simu_params],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    reachy_sdk_stuff = [
        Node(
            package='reachy_kinematics',
            executable='arm_kinematics_service',
        ),
        Node(
            package='reachy_controllers',
            executable='mockup_controller',
        ),
        Node(
            package='reachy_sdk_server',
            executable='reachy_sdk_server',
        ),
    ]

    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawn_entity,
        #         on_exit=[load_joint_state_controller],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_controller,
        #         on_exit=[load_joint_trajectory_controller],
        #     )
        # ),
        spawn_entity,
        controller_manager_node,
        spawn_jsb_controller,
        spawn_forward_controller,
        gazebo,
        control_node,
        node_robot_state_publisher,
        static_tf,
    ] + reachy_sdk_stuff)
