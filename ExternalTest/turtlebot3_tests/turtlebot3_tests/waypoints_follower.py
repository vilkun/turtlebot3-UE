#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.srv import ManageLifecycleNodes

"""
Ref: https://github.com/ros-planning/navigation2/blob/main/nav2_system_tests/src/waypoint_follower/tester.py
"""
class WaypointsFollower(Node):
    def __init__(self, in_node_name):
        super().__init__(node_name=in_node_name, namespace='')
        
        self.declare_parameter("waypoints", [])
        self.declare_parameter("initial_pose", [])

        waypoints = []
        wps_param = self.get_parameter("waypoints").value
        print(wps_param)
        for wp in wps_param:
            wp_vals = wp.split(',')
            waypoints.append([float(wp_vals[0]), float(wp_vals[1])])
        self.set_waypoints(waypoints)
        self.action_client = ActionClient(self, FollowWaypoints, 'FollowWaypoints')

        self.initial_pose = self.get_parameter("initial_pose").value        
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose_pub', 10)
        self.initial_pose_received = False
        self.publish_initial_pose(self.initial_pose)
        
        self.goal_handle = None

        pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
          history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
          depth=1)

        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                       'amcl_pose', self.pose_callback, pose_qos)

    def __enter__(self):
        return self

    def __exit__(self, exep_type, exep_value, trace):
        if exep_type is not None:
            raise Exception('Exception occured, value: ', exep_value)
        self.shutdown()

    def shutdown(self):
        #self.shutdown_nav_lifecycle()
        self.destroy_node()

    def publish_initial_pose(self, in_pose):
        assert(len(in_pose) >= 7)
        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose.pose.pose.position.x = in_pose[0]
        self.init_pose.pose.pose.position.y = in_pose[1]
        self.init_pose.pose.pose.position.z = in_pose[2]
        self.init_pose.pose.pose.orientation.x = in_pose[3]
        self.init_pose.pose.pose.orientation.y = in_pose[4]
        self.init_pose.pose.pose.orientation.z = in_pose[5]
        self.init_pose.pose.pose.orientation.w = in_pose[6]
        self.init_pose.header.frame_id = 'map'
        self.initial_pose_pub.publish(self.init_pose)
        time.sleep(5)

    def pose_callback(self, msg):
        self.info_msg('Received amcl_pose')
        self.initial_pose_received = True

    def set_waypoints(self, in_waypoints):
        self.waypoints = []

        for wp in in_waypoints:
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.x = wp[0]
            msg.pose.position.y = wp[1]
            msg.pose.orientation.w = 1.0
            self.waypoints.append(msg)

    def follow_waypoints(self):
        # Wait for the whole nav2 stack to be setup
        time.sleep(10)
        retry_count = 0
        retries = 2
        while not self.initial_pose_received and retry_count <= retries:
            retry_count += 1
            self.info_msg(f'Setting initial pose {self.initial_pose}')
            self.publish_initial_pose(self.initial_pose)
            self.info_msg('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1.0)  # wait for pose_callback

        result = self.run(block=True)
        if not result:
            self.error_msg('Following waypoints FAILED')
        else:
            self.info_msg('Following waypoints PASSED')
        return result

    def run(self, block):
        if not self.waypoints:
            rclpy.error_msg('Did not set valid waypoints before running test!')
            return False

        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.info_msg("'FollowWaypoints' action server not available, waiting...")

        action_request = FollowWaypoints.Goal()
        action_request.poses = self.waypoints

        self.info_msg('Sending goal request...')
        send_goal_future = self.action_client.send_goal_async(action_request)
        try:
            rclpy.spin_until_future_complete(self, send_goal_future)
            self.goal_handle = send_goal_future.result()
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

        if not self.goal_handle.accepted:
            self.error_msg('Goal rejected')
            return False

        self.info_msg('Goal accepted')
        if not block:
            return True

        get_result_future = self.goal_handle.get_result_async()

        self.info_msg("Waiting for 'FollowWaypoints' action to complete")
        try:
            rclpy.spin_until_future_complete(self, get_result_future)
            status = get_result_future.result().status
            result = get_result_future.result().result
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.error_msg('Goal failed with status code: {0}'.format(status))
            return False

        if len(result.missed_waypoints) > 0:
            self.warn_msg('Goal failed to process all waypoints,'
                          ' missed {0} wps.'.format(len(result.missed_waypoints)))
        self.info_msg('Goal succeeded!')
        return True

    def shutdown_nav_lifecycle(self):
        self.info_msg('Nav life cycle shutting down')
        transition_service = 'lifecycle_manager_navigation/manage_nodes'
        mgr_client = self.create_client(ManageLifecycleNodes, transition_service)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(transition_service + ' service not available, waiting...')

        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request().SHUTDOWN
        future = mgr_client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future)
            future.result()
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

        transition_service = 'lifecycle_manager_localization/manage_nodes'
        mgr_client = self.create_client(ManageLifecycleNodes, transition_service)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(transition_service + ' service not available, waiting...')

        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request().SHUTDOWN
        future = mgr_client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future)
            future.result()
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

    def cancel_goal(self):
        cancel_future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)

    def info_msg(self, msg: str):
        self.get_logger().info(msg)

    def warn_msg(self, msg: str):
        self.get_logger().warn(msg)

    def error_msg(self, msg: str):
        self.get_logger().error(msg)

if __name__ == '__main__':
    rclpy.init()
    with WaypointsFollower(in_node_name='tb3_nav2_waypointfollower') as wps_follower:
        wps_follower.follow_waypoints()
    rclpy.shutdown()
    