#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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
#
# Modified by AutomaticAddison.com

from geometry_msgs.msg import PoseStamped, PoseArray # Pose with ref frame and timestamp
from std_msgs.msg import String
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # Helper module

from enum import Enum

class Status(Enum):
    WAITING = 1
    DRIVEING = 2
    ARRIVE = 3


class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        # Launch the ROS 2 Navigation Stack
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        self.tail_navigator = BasicNavigator()
        self.tail_navigator.waitUntilNav2Active()

        self.head_timer = self.create_timer(0.04, self.head_run)
        self.tail_timer = self.create_timer(0.04, self.tail_run)

        self.waypoints_sub = self.create_subscription(PoseArray, 'waypoints_web', self.waypoint_callback, 1)
        self.head_goal_pose_sub = self.create_subscription(PoseStamped, 'omo/goal_pose_web', self.head_goal_pose_callback, 1)
        self.tail_goal_pose_sub = self.create_subscription(PoseStamped, 'goal_pose_web', self.tail_goal_pose_callback, 1)
        self.tracking_mode_pub = self.create_publisher(String, "tracking_mode", 1)
        self.waypoints = list()
        self.head_goal_pose = None
        self.tail_goal_pose = None
        self.head_state = Status.WAITING
        self.tail_state = Status.WAITING

        self.tail_mode = "tracking"

    def waypoint_callback(self, msg):
        if not self.waypoints:
            for pose in msg.poses:
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                goal_pose.pose = pose

                self.waypoints.append(goal_pose)
    
    def head_goal_pose_callback(self, msg):
        if self.head_goal_pose is None:
            print("Head goal pose received.")
            self.head_goal_pose = msg
    
    def tail_goal_pose_callback(self, msg):
        if self.tail_goal_pose is None:
            print("Tail goal pose received.")
            self.tail_goal_pose = msg

    def waypoint_navigation(self):
        if not self.waypoints: return

        nav_start = self.navigator.get_clock().now()
        self.navigator.followWaypoints(self.waypoints)

        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(self.waypoints)))
                now = self.navigator.get_clock().now()

                # Some navigation timeout to demo cancellation
                if now - nav_start > Duration(seconds=100000000.0):
                    self.navigator.cancelTask()

                # Some follow waypoints request change to demo preemption
                if now - nav_start > Duration(seconds=500000.0):
                    goal_pose_alt = PoseStamped()
                    goal_pose_alt.header.frame_id = 'map'
                    goal_pose_alt.header.stamp = now.to_msg()
                    goal_pose_alt.pose = self.waypoints[0].pose
                    self.waypoints = [goal_pose_alt]
                    nav_start = now
                    self.navigator.followWaypoints(self.waypoints)

        self.waypoints.clear()
    
    def nav_to_pose(self, goal_pose):
        if goal_pose is None: return

        self.navigator.goToPose(goal_pose)

        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                      + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                    goal_pose.pose.position.x = -3.0
                    self.navigator.goToPose(goal_pose)

    def head_run(self):
        if self.waypoints:
            self.waypoint_navigation()
        elif self.head_goal_pose:
            self.nav_to_pose(self.head_goal_pose)
            self.head_goal_pose = None
        else:
            return

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            # self.tail_mode = "navigating"
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
    
    def tail_run(self):
        msg = String()
        msg.data = self.tail_mode
        self.tracking_mode_pub.publish(msg)
        if self.tail_mode == "navigating":
            if self.tail_goal_pose:
                self.nav_to_pose(self.tail_goal_pose)
                self.tail_goal_pose = None
            else:
                return

            # Do something depending on the return code
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
                ## 수취 확인하면 head를 움직임, 이때의 tail_state 기록 필요
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')


            
            
            

        
                
                

'''
Follow waypoints using the ROS 2 Navigation Stack (Nav2)
'''
def main(args=None):
    rclpy.init(args=args)

    navigation_client = NavigationClient()

    rclpy.spin(navigation_client)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigation_client.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
