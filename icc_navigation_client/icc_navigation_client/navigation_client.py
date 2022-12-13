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

from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from std_msgs.msg import Empty
from icc_navigation_interfaces.msg import RobotStatus
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from rclpy.node import Node
from icc_navigation_client.robot_navigator import BasicNavigator, TaskResult # Helper module


class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        # Launch the ROS 2 Navigation Stack
        self.head_navigator = BasicNavigator("omo")
        self.head_navigator.waitUntilNav2Active()

        self.tail_navigator = BasicNavigator("")
        self.tail_navigator.waitUntilNav2Active()

        self.head_timer = self.create_timer(0.04, self.head_run)
        # self.tail_timer = self.create_timer(0.04, self.tail_run)
        # self.tf_timer = self.create_timer(0.04, self.tf_run) ## marker tf 0.5m 뒤에표시할것

        self.head_goal_pose_sub = self.create_subscription(PoseStamped, 'omo/goal_pose_web', self.head_goal_pose_callback, 1)
        self.tail_goal_pose_sub = self.create_subscription(PoseStamped, 'goal_pose_web', self.tail_goal_pose_callback, 1)
        self.reception_sub = self.create_subscription(Empty, 'reception', self.reception_callback, 1)
        self.head_mode_pub = self.create_publisher(RobotStatus, "head_mode", 1)
        self.tail_mode_pub = self.create_publisher(RobotStatus, "tail_mode", 1)
        self.head_goal_pose = None
        self.head_goal_pose = PoseStamped()
        self.head_goal_pose.pose.position.x = 0
        self.head_goal_pose.pose.position.y = 0
        self.head_goal_pose.pose.position.z = 0
        self.tail_goal_pose = None
        self.head_mode = RobotStatus.STATUS_WAITING
        self.tail_mode = RobotStatus.STATUS_TRACKING
        self.reception = False
    
    def head_goal_pose_callback(self, msg):
        if self.head_goal_pose is None:
            print("Head goal pose received.")
            self.head_goal_pose = msg
            if self.head_mode == RobotStatus.STATUS_WAITING and self.tail_mode == RobotStatus.STATUS_TRACKING:
                self.head_mode = RobotStatus.STATUS_DRIVING
    
    def tail_goal_pose_callback(self, msg):
        if self.tail_goal_pose is None:
            print("Tail goal pose received.")
            self.tail_goal_pose = msg
            if self.tail_mode == RobotStatus.STATUS_TRACKING:
                self.tail_mode = RobotStatus.STATUS_DRIVING
    
    def reception_callback(self, msg):
        if not self.reception:
            self.tail_mode_pub.publish(RobotStatus(state=RobotStatus.STATUS_DEPART_BACK))
            self.tail_mode = RobotStatus.STATUS_DRIVING_BACK
            self.reception = True
    
    def nav_to_pose(self, navigator, goal_pose):
        if goal_pose is None: return

        navigator.goToPose(goal_pose)

        i = 0
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                      + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                    goal_pose.pose.position.x = -3.0
                    navigator.goToPose(goal_pose)

    def head_run(self):
        if self.head_goal_pose is not None:
            self.nav_to_pose(self.head_navigator, self.head_goal_pose)
            self.head_goal_pose = None
        else:
            return

        # Do something depending on the return code
        result = self.head_navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            if self.head_mode == RobotStatus.STATUS_DRIVING:
                self.head_mode_pub.publish(RobotStatus(state=RobotStatus.STATUS_ARRIVE))
                self.head_mode_pub.publish(RobotStatus(state=RobotStatus.STATUS_WAITING))
                self.tail_mode_pub.publish(RobotStatus(state=RobotStatus.STATUS_DEPART))
                self.head_mode = RobotStatus.STATUS_WAITING
            elif self.head_mode == RobotStatus.STATUS_DEPART_BACK:
                self.head_mode_pub.publish(RobotStatus(state=RobotStatus.STATUS_ARRIVE))
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
    
    # def tail_run(self):
    #     if self.tail_goal_pose is not None:
    #         self.nav_to_pose(self.tail_navigator, self.tail_goal_pose)
    #         self.tail_goal_pose = None
    #     else:
    #         return

    #     # Do something depending on the return code
    #     result = self.tail_navigator.getResult()
    #     if result == TaskResult.SUCCEEDED:
    #         print('Goal succeeded!')
    #         ## 수취 확인하면 head를 움직임, 이때의 tail_state 기록 필요
    #         if self.tail_mode == RobotStatus.STATUS_DRIVING:
    #             self.tail_mode_pub.publish(RobotStatus(state=RobotStatus.STATUS_ARRIVE))
    #             self.tail_mode_pub.publish(RobotStatus(state=RobotStatus.STATUS_WAITING))
    #             self.tail_mode = RobotStatus.STATUS_WAITING
    #         elif self.tail_mode == RobotStatus.STATUS_DRIVING_BACK:
    #             self.tail_mode_pub.publish(RobotStatus(state=RobotStatus.STATUS_ARRIVE_BACK))
    #             self.head_mode_pub.publish(RobotStatus(state=RobotStatus.STATUS_DEPART_BACK))
    #             self.tail_mode = RobotStatus.STATUS_TRACKING
    #             self.head_mode = RobotStatus.STATUS_DEPART_BACK
    #             self.reception = False
    #     elif result == TaskResult.CANCELED:
    #         print('Goal was canceled!')
    #     elif result == TaskResult.FAILED:
    #         print('Goal failed!')
    #     else:
    #         print('Goal has an invalid return status!')

'''
Follow waypoints using the ROS 2 Navigation Stack (Nav2)
'''
def main(args=None):
    rclpy.init(args=args)

    navigation_client = NavigationClient()

    rclpy.spin(navigation_client)

    navigation_client.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
