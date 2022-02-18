#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

# Refs:
# https://github.com/ros2/launch/blob/master/launch_testing/test/launch_testing/examples/args_launch_test.py
# https://docs.ros.org/en/galactic/How-To-Guides/Launch-file-different-formats.html

import os
import time
import unittest
from ament_index_python import get_package_share_directory

import launch
import launch.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import launch_testing.util
import pytest

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

"""
Test basic navigation going to goal pose
Ref: https://github.com/ros-planning/navigation2/blob/main/nav2_simple_commander/nav2_simple_commander/example_nav_to_pose.py
"""

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # [tb3_model] arg
    tb3_model = launch.substitutions.LaunchConfiguration('tb3_model', default='burger')

    # Bringup the turtlebot3
    tb3_robot_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'launch/robot.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Start tb3 nav2
    tb3_nav2_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch/tb3_simulation_launch.py')),
        launch_arguments={'map':'maps/Turtlebot3_benchmark.yaml'}.items()
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'tb3_model',
            default_value=tb3_model,
            description='turtlebot3 model (burger or waffle)'),
        launch.actions.SetEnvironmentVariable('TURTLEBOT3_MODEL', tb3_model),
        tb3_robot_launch,
        tb3_nav2_launch,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest()
    ])

class TestGoalReach(unittest.TestCase):
    def test_goal_reach(self, proc_output):
        rclpy.init()
        assert reach_goal(60.0), 'Goal pose failed being reached!'
        rclpy.shutdown()

def gen_goal_pose(navigator, pos_x, pos_y, orient_z, orient_w):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = pos_x
    goal_pose.pose.position.y = pos_y

    goal_pose.pose.orientation.z = orient_z
    goal_pose.pose.orientation.w = orient_w
    return goal_pose

def reach_goal(timeout):
    navigator = BasicNavigator()

    # Set initial pose
    initial_pose = gen_goal_pose(navigator, 0.0, 0.0, 1.0, 0.0)
    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    navigator.lifecycleStartup()

    # If autostarting nav2, wait for navigation to fully activate, 
    #navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Set goal pose
    goal_pose = gen_goal_pose(navigator, 2.0, 3.0, 0.0, 1.0)

    # sanity check a valid path exists
    path = navigator.getPath(initial_pose, goal_pose)
    if path is None:
        print(f"NOT FOUND: Path from initial pose[{initial_pose.pose}] to goal pose[{goal_pose.pose}]!")
        return -1

    print(f"FOUND: Path from initial pose[{initial_pose.pose}] to goal pose[{goal_pose.pose}]!")
    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=timeout):
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=0.7*timeout):
                goal_pose.pose.position.x = -3.0
                navigator.goToPose(goal_pose)

    # Do something depending on the return code
    result = navigator.getResult()
    if TaskResult.SUCCEEDED == result:
        print('Goal succeeded!')
    elif TaskResult.CANCELED == result:
        print('Goal was canceled!')
    elif TaskResult.FAILED == result:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    return 0 if (result == TaskResult.SUCCEEDED) else -1
