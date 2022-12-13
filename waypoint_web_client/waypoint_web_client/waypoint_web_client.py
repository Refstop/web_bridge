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
from marker_docking_interfaces.msg import DockingMode
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # Helper module

class WaypointWebClient(Node):
    def __init__(self):
        super().__init__('waypoint_web_client')
        # Launch the ROS 2 Navigation Stack
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        self.timer = self.create_timer(0.04, self.run)

        self.create_subscription(PoseArray, 'waypoints_web', self.waypoint_callback, 1)
        self.create_subscription(DockingMode, 'docking_mode', self.docking_mode_callback, 1)
        self.docking_mode_pub = self.create_publisher(DockingMode, 'docking_mode', 1)
        self.goal_poses = list()
        self.goal_poses_last = None

        self.docking_complete = False

    def waypoint_callback(self, msg):
        if not self.goal_poses:
            for pose in msg.poses:
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                goal_pose.pose = pose
                self.goal_poses.append(goal_pose)

    def docking_mode_callback(self, msg):
        if msg.state == DockingMode.STATUS_COMPLETE:
            self.docking_complete = True
            self.goal_poses.append(self.goal_poses_last)

    def run(self):
        if not self.goal_poses: return

        nav_start = self.navigator.get_clock().now()
        self.navigator.followWaypoints(self.goal_poses)

        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(self.goal_poses)))

                now = self.navigator.get_clock().now()
                # Some navigation timeout to demo cancellation
                if now - nav_start > Duration(seconds=180.0):
                    self.navigator.cancelTask()

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            self.goal_poses_last = self.goal_poses[-1]
            self.goal_poses.clear()
            if not self.docking_complete:
                self.docking_mode_pub.publish(DockingMode(state=DockingMode.STATUS_DOCKING))
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        if self.docking_complete:
            self.docking_complete = False
            self.goal_poses_last = None
'''
Follow waypoints using the ROS 2 Navigation Stack (Nav2)
'''
def main(args=None):
    rclpy.init(args=args)

    waypoint_web_client = WaypointWebClient()

    rclpy.spin(waypoint_web_client)

    waypoint_web_client.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()