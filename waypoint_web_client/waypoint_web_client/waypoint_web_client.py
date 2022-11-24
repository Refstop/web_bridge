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

import time # Time library

from geometry_msgs.msg import PoseStamped, PoseArray # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # Helper module
import numpy as np # Scientific computing library for Python

class WaypointWebClient(Node):
    def __init__(self):
        super().__init__('waypoint_web_client')
        # Launch the ROS 2 Navigation Stack
        self.navigator = BasicNavigator()

        # Set the robot's initial pose if necessary
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.t = None
        self.init = False

        self.timer = self.create_timer(0.5, self.run)

        self.subscription = self.create_subscription(PoseArray, 'waypoints_web', self.waypoint_callback, 1)
        self.goal_poses = list()

    def waypoint_callback(self, msg):
        if not self.goal_poses:
            for pose in msg.poses:
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                goal_pose.pose = pose

                self.goal_poses.append(goal_pose)

    def run(self):
        if self.t == None:
            try:
                self.t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            except TransformException as ex:
                print('Could not transform map to base_link:', ex)
        else:
            if not self.init:
                initial_pose = PoseStamped()
                initial_pose.header.frame_id = 'map'
                initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                initial_pose.pose.position.x = self.t.transform.translation.x
                initial_pose.pose.position.y = self.t.transform.translation.y
                initial_pose.pose.position.z = 0.0

                initial_pose.pose.orientation.x = self.t.transform.rotation.x
                initial_pose.pose.orientation.y = self.t.transform.rotation.y
                initial_pose.pose.orientation.z = self.t.transform.rotation.z
                initial_pose.pose.orientation.w = self.t.transform.rotation.w
                self.navigator.setInitialPose(initial_pose)

                self.navigator.waitUntilNav2Active()
                self.init = True
            else:
                if not self.goal_poses: return
                # If desired, you can change or load the map as well
                # navigator.changeMap('/path/to/map.yaml')

                # You may use the navigator to clear or obtain costmaps
                # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
                # global_costmap = navigator.getGlobalCostmap()
                # local_costmap = navigator.getLocalCostmap()
                
                # sanity check a valid path exists
                # path = navigator.getPathThroughPoses(initial_pose, goal_poses)

                nav_start = self.navigator.get_clock().now()
                self.navigator.followWaypoints(self.goal_poses)

                i = 0
                while not self.navigator.isTaskComplete():
                    ################################################
                    #
                    # Implement some code here for your application!
                    #
                    ################################################

                    # Do something with the feedback
                    i = i + 1
                    feedback = self.navigator.getFeedback()
                    if feedback and i % 5 == 0:
                        print('Executing current waypoint: ' +
                            str(feedback.current_waypoint + 1) + '/' + str(len(self.goal_poses)))
                        now = self.navigator.get_clock().now()

                        # Some navigation timeout to demo cancellation
                        if now - nav_start > Duration(seconds=100000000.0):
                            self.navigator.cancelTask()

                        # Some follow waypoints request change to demo preemption
                        if now - nav_start > Duration(seconds=500000.0):
                            goal_pose_alt = PoseStamped()
                            goal_pose_alt.header.frame_id = 'map'
                            goal_pose_alt.header.stamp = now.to_msg()
                            goal_pose_alt.pose = self.goal_poses[0].pose
                            self.goal_poses = [goal_pose_alt]
                            nav_start = now
                            self.navigator.followWaypoints(self.goal_poses)

                # Do something depending on the return code
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    print('Goal succeeded!')
                elif result == TaskResult.CANCELED:
                    print('Goal was canceled!')
                elif result == TaskResult.FAILED:
                    print('Goal failed!')
                else:
                    print('Goal has an invalid return status!')
                
                self.goal_poses.clear()
                # exit(0)
                # navigator.lifecycleShutdown()

'''
Follow waypoints using the ROS 2 Navigation Stack (Nav2)
'''
def main(args=None):
    rclpy.init(args=args)

    waypoint_web_client = WaypointWebClient()

    rclpy.spin(waypoint_web_client)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waypoint_web_client.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
