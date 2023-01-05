import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Vector3
from ros2_aruco_interfaces.msg import ArucoMarkers
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from marker_docking_interfaces.msg import DockingMode

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import math
import cv2
import numpy as np
from marker_docking import transformations
from enum import Enum

rad2deg = 180/math.pi

class CalibMode(Enum):
    ROTATE = 1
    STRAIGHT = 2
    FACE2FACE = 3
    READY2ALIGN = 4

class MarkerDocking(Node):
    def __init__(self):
        super().__init__('marker_docking')
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 1)
        self.docking_mode_pub = self.create_publisher(DockingMode, "docking_mode", 1)
        self.create_subscription(AprilTagDetectionArray, 'apriltag/detections', self.april_callback, 1)
        self.create_subscription(DockingMode, 'docking_mode', self.docking_callback, 1)
        self.april_timer = self.create_timer(0.04, self.april_run)
        self.calib_timer = self.create_timer(0.04, self.calib_run)
        self.align_timer = self.create_timer(0.04, self.align_run)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.docking = None
        # self.docking = DockingMode.STATUS_DOCKING
        self.dist_virtual = 1e3
        self.target = None
        self.calib_init = False
        self.ids = list()
        self.step = 0
        # self.step = CalibMode.READY2ALIGN

    def april_callback(self, msg):
        for detection in msg.detections:
            self.center_offset = 0
            self.ids.append(detection.id)

    def docking_callback(self, msg):
        self.get_logger().info("receive!")
        if msg.state == DockingMode.STATUS_DOCKING:
            self.docking = DockingMode.STATUS_DOCKING

    def euclidean_distance(self, p1, p2):
        return math.sqrt((p1.transform.translation.x - p2.transform.translation.x)**2 + (p1.transform.translation.y - p2.transform.translation.y)**2)

    def april_run(self):
        thres_dist = 0.3
        for id in self.ids:
            if id == 0 or id == 1:
                try:
                    t = self.tf_buffer.lookup_transform('camera_frame', 'tag36h11:'+str(id), rclpy.time.Time())
                    t_rvs = self.tf_buffer.lookup_transform('tag36h11:'+str(id), 'camera_frame', rclpy.time.Time())
                except TransformException as ex:
                    print('Could not transform odom to base_link:', ex)
                    return
                T_cm = transformations.quaternion_matrix([t.transform.rotation.x,
                                                       t.transform.rotation.y,
                                                       t.transform.rotation.z,
                                                       t.transform.rotation.w])
                trans = np.array([t.transform.translation.x,
                              t.transform.translation.y,
                              t.transform.translation.z])
                T_cm[0:3, 3] = trans
                TT = np.array([0, 0, thres_dist, 1])
                T_virtual = np.matmul(T_cm, TT)

                # euler_rad = -cv2.decomposeProjectionMatrix(T_cm[:3,:])[6]
                # euler_rad *= math.pi/180
                # self.angular_euler = -euler_rad[1][0] # theta_m
                self.angular_euler = math.atan2(t_rvs.transform.translation.x, t_rvs.transform.translation.z)
                
                dist_real = math.sqrt(T_cm[0][3] ** 2 + T_cm[2][3] ** 2)
                self.dist_virtual = math.sqrt(T_virtual[0] ** 2 + T_virtual[2] ** 2)
                if self.angular_euler > 0:
                    th_c = math.pi/2 - self.angular_euler
                    self.theta_back = math.pi/2
                else:
                    th_c = -self.angular_euler - math.pi/2
                    self.theta_back = -math.pi/2
                
                if not self.calib_init and self.docking == None:
                    self.dist_c = dist_real * math.cos(abs(th_c))
                    self.get_logger().info("self.angular_euler: " + str(self.angular_euler*rad2deg))
                    self.get_logger().info("th_c: " + str(th_c*rad2deg))
                    self.angular_pre = th_c + math.atan2(T_cm[0][3], T_cm[2][3])
                    self.calib_init = True

                self.angular = math.atan2(T_virtual[0], T_virtual[2])
                # self.get_logger().info("self.angular_euler: " + str(self.angular_euler))
                # self.get_logger().info("self.angular_pre: " + str(self.angular_pre))

    def calib_run(self):
        if not self.calib_init: return
        const_vel = 0.1
        const_ang = 0.2
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except TransformException as ex:
            print('Could not transform odom to base_link:', ex)
            return
        
        if self.target is None:
            if self.step == 0:
                self.step = CalibMode.ROTATE
                yaw = transformations.euler_from_quaternion([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])[2]
                self.target = self.angular_pre + yaw
                
                self.get_logger().info("yaw: " + str(yaw*rad2deg))
                self.get_logger().info("self.angular_pre: " + str(self.angular_pre*rad2deg))
                self.get_logger().info("self.target: " + str(self.target*rad2deg))
                # return
            elif self.step == CalibMode.STRAIGHT:
                t_mat = transformations.quaternion_matrix([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
                t_mat[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
                dist_c_mat = transformations.translation_matrix([self.dist_c, 0.0, 0.0])
                self.target = t_mat@dist_c_mat

            elif self.step == CalibMode.FACE2FACE:
                yaw = transformations.euler_from_quaternion([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])[2]
                self.target = self.theta_back + yaw
    
        if self.step == CalibMode.ROTATE:
            yaw = transformations.euler_from_quaternion([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])[2]
            # self.get_logger().info("target, yaw: " + str(self.target*rad2deg) + ' ' + str(yaw*rad2deg))
            error = self.target - yaw
            # self.get_logger().info("error: " + str(error*rad2deg))
            if error < 0:
                const_ang *= -1
            self.cmd_vel_pub.publish(Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=const_ang)))
            if abs(error) < 0.05:
                self.step = CalibMode.STRAIGHT
                self.target = None
        elif self.step == CalibMode.STRAIGHT:
            error = math.sqrt((t.transform.translation.x - self.target[0][3])**2 + (t.transform.translation.y - self.target[1][3])**2)
            self.cmd_vel_pub.publish(Twist(linear=Vector3(x=const_vel, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)))
            if abs(error) < 0.02:
                self.step = CalibMode.FACE2FACE
                self.target = None
        elif self.step == CalibMode.FACE2FACE:
            yaw = transformations.euler_from_quaternion([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])[2]
            error = self.target - yaw
            self.get_logger().info("error: " + str(error*rad2deg))
            if error < 0:
                const_ang *= -1
            self.cmd_vel_pub.publish(Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=const_ang)))
            if abs(error) < 0.05:
                self.step = CalibMode.READY2ALIGN
                self.target = None
    
    def align_run(self):
        if self.dist_virtual == 1e3 or self.step != CalibMode.READY2ALIGN: return
        self.get_logger().info("Align Start!")
        const_vel = 0.04
        const_ang = 0.05
        if self.docking == DockingMode.STATUS_DOCKING:
            if self.dist_virtual <= 0.03: # 2. drived in sufficient distance, stop(will substitute to work function), when finished entering to unloading mode
                self.linear_x = 0.0
                self.angular_z = 0.0
                inp = str(input("input mode(d: docking, u: undocking): "))
                if inp == "u":
                    self.docking = DockingMode.STATUS_UNDOCKING
            elif self.dist_virtual <= 0.15:
                self.linear_x = const_vel
                self.angular_z = self.angular_euler
            else: # 1. drive until setting distance
                self.linear_x = const_vel
                self.angular_z = self.angular
            self.cmd_vel_pub.publish(Twist(linear=Vector3(x=self.linear_x, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-self.angular_z)))
        elif self.docking == DockingMode.STATUS_UNDOCKING: # undocking
            if self.dist_virtual < 0.45: # 3. drive backward
                self.linear_x = -const_vel
                self.angular_z = self.angular_euler
            else: # 5. stop and change start, obsv, unloading signal to false & waypoint publish
                self.linear_x = 0.0
                self.angular_z = 0.0
                self.docking = None
                self.step = 0
                self.docking_mode_pub.publish(DockingMode(state=DockingMode.STATUS_COMPLETE))
                inp = str(input("input mode(d: docking, u: undocking): "))
                if inp == "d":
                    self.docking = DockingMode.STATUS_DOCKING
                # exit()
            self.cmd_vel_pub.publish(Twist(linear=Vector3(x=self.linear_x, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-self.angular_z)))
        

def main(args=None):
    rclpy.init(args=args)
    marker_docking = MarkerDocking()

    rclpy.spin(marker_docking)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
