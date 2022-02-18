#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

import os
import sys
import time
import unittest
import ament_index_python
from ament_index_python import get_package_share_directory

import launch
import launch.actions
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import launch_testing.markers

import rclpy
from rclpy.node import Node

import pytest

"""
Test basic navigation following waypoints
"""

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    wps_follower_params = PathJoinSubstitution(
        [
            FindPackageShare('turtlebot3_tests'),
            'config',
            'test_waypoint_follower.yaml'
        ]
    )

    # [tb3_model] arg
    tb3_model = launch.substitutions.LaunchConfiguration('tb3_model', default='burger')

    # Bringup the turtlebot3
    tb3_robot_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'launch/robot.launch.py')),
        launch_arguments={'use_sim_time': 'False'}.items()
    )

    # Start tb3 nav2
    tb3_nav2_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch/tb3_simulation_launch.py')),
        launch_arguments={'use_simulator': 'False', 'headless': 'True', 'map':'maps/Turtlebot3_benchmark.yaml'}.items()
    )
        
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'tb3_model',
            default_value=tb3_model,
            description='turtlebot3 model (burger or waffle)'),
        launch.actions.SetEnvironmentVariable('TURTLEBOT3_MODEL', tb3_model),
        tb3_robot_launch,
        tb3_nav2_launch,
        launch_ros.actions.Node(
            package = 'turtlebot3_tests',
            name = 'tb3_nav2_waypointfollower',
            executable = 'waypoints_follower.py',
            parameters = [wps_follower_params],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
        ),
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest()
    ])

class TestWaypointFollower(unittest.TestCase):
    def test_waypoint_follower(self, proc_output):
        rclpy.init()
        node = Node('test_wps_follower_node')
        assert wait_for_node(node, 'tb3_nav2_waypointfollower', timeout=8.0), '[tb3_nav2_waypointfollower] node not found!'
        rclpy.shutdown()

def wait_for_node(in_waiter_node, target_node_name, timeout=8.0):
    start = time.time()
    flag = False
    print(f'Waiting for node {target_node_name}...')
    while time.time() - start < timeout and not flag:
        flag = target_node_name in in_waiter_node.get_node_names()
        time.sleep(0.1)
    return flag
