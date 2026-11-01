#!/usr/bin/env python3
"""Create 3 LiDAR mapping launch (portfolio copy).

The supplied demonstration file contained duplicate module/function definitions.
This file retains its *second*, effective generate_launch_description() definition.
An external create3_lidar_slam package with mapper_params_online_async.yaml
is required. Check the LiDAR mounting transform before operating the robot.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    rplidar_launch = os.path.join(
        get_package_share_directory('rplidar_ros'), 'launch', 'rplidar.launch.py'
    )
    slam_params = os.path.join(
        get_package_share_directory('create3_lidar_slam'),
        'config', 'mapper_params_online_async.yaml'
    )

    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch)
    )

    # The LiDAR is centered and aligned with the robot.  z=0 is a temporary
    # value for TF connectivity tests; replace it with the measured height
    # relative to the base_link origin before collecting a final map.
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_static_tf',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'laser',
        ],
        remappings=[('/tf_static', '/robot_1/tf_static')],
    )

    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {
                'use_sim_time': False,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',  # base_footprint does not exist here
                'scan_topic': '/scan',      # actual topic from RPLIDAR
                'mode': 'mapping',
            },
        ],
        remappings=[
            ('/tf', '/robot_1/tf'),
            ('/tf_static', '/robot_1/tf_static'),
        ],
    )

    # In Jazzy, SLAM Toolbox is a lifecycle node, so configure and activate it.
    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )
    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    return LaunchDescription([
        lidar_driver,
        laser_tf,
        slam,
        activate_slam,
        configure_slam,
    ])
