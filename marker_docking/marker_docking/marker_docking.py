import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Vector3
from ros2_aruco_interfaces.msg import ArucoMarkers
from marker_docking_interfaces.msg import DockingMode

import math
import cv2
import numpy as np
# from pynput import keyboard

def euler_from_quaternion(x, y, z, w):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def rot3d(roll, pitch, yaw):
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), np.sin(roll)],
                    [0, -np.sin(roll), np.cos(roll)]])
    R_y = np.array([[np.cos(pitch), 0, -np.sin(pitch)],
                    [0, 1, 0],
                    [np.sin(pitch), 0, np.cos(pitch)]])
    R_z = np.array([[np.cos(yaw), np.sin(yaw), 0],
                    [-np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1],])

    return np.matmul(R_z, R_y, R_x)


class MarkerDocking(Node):
    def __init__(self):
        super().__init__('marker_docking')
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 1)
        self.docking_mode_pub = self.create_publisher(DockingMode, "docking_mode", 1)
        self.create_subscription(ArucoMarkers, 'fiducial_transforms', self.aruco_callback, 1)
        self.create_subscription(DockingMode, 'docking_mode', self.docking_callback, 1)
        self.timer = self.create_timer(0.04, self.run)

        # self.keylistener = keyboard.Listener(on_release=self.on_release)
        # self.keylistener.start()
        self.docking = DockingMode.STATUS_DOCKING
        self.distance = 1e3

    def aruco_callback(self, msg):
        # roll, pitch, yaw = euler_from_quaternion(msg.markers[0].transform.rotation.x,
        #                                          msg.markers[0].transform.rotation.y,
        #                                          msg.markers[0].transform.rotation.z,
        #                                          msg.markers[0].transform.rotation.w)
        # R = rot3d(roll, pitch, yaw)
        R = transformations.quaternion_matrix([msg.markers[0].transform.rotation.x,
                                               msg.markers[0].transform.rotation.y,
                                               msg.markers[0].transform.rotation.z,
                                               msg.markers[0].transform.rotation.w])
        t = np.array([msg.markers[0].transform.translation.x,
                      msg.markers[0].transform.translation.y,
                      msg.markers[0].transform.translation.z])
        hs = np.hstack([R, t.reshape(3,1)])
        T_cm = np.vstack([hs, np.array([0, 0, 0, 1])])
        TT = np.array([0, 0, 0.5, 1])
        T_virtual = np.matmul(T_cm, TT)

        euler_rad = -cv2.decomposeProjectionMatrix(hs)[6]
        euler_rad = euler_rad * math.pi/180
        self.distance = math.sqrt(T_virtual[0] ** 2 + T_virtual[2] ** 2)
        self.angular_euler = -euler_rad[1][0] # ??
        self.angular = math.atan2(T_virtual[0], T_virtual[2])

    def docking_callback(self, msg):
        if msg.state == DockingMode.DOCKING:
            self.docking = DockingMode.STATUS_DOCKING

    # def on_release(self, key):
    #     if key == keyboard.Key.enter:
    #         self.docking = DockingMode.STATUS_UNDOCKING

    def run(self):
        if self.distance == 1e3: return
        if self.docking == DockingMode.STATUS_DOCKING:
            if self.distance <= 0.5: # 2. drived in sufficient distance, stop(will substitute to work function), when finished entering to unloading mode
                self.linear_x = 0
                self.angular_z = 0
            elif self.distance <= 0.15:
                self.linear_x = 0.08
                self.angular_z = self.angular_euler
            else: # 1. drive until setting distance
                self.linear_x = 0.08
                self.angular_z = self.angular
            self.cmd_vel_pub.publish(Twist(linear=Vector3(x=self.linear_x, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-self.angular_z)))
        elif self.docking == DockingMode.STATUS_UNDOCKING: # undocking
            if self.distance <  0.45: # 3. drive backward
                self.linear_x = -0.08
                self.angular_z = self.angular_euler
            else: # 5. stop and change start, obsv, unloading signal to false & waypoint publish
                self.linear_x = 0
                self.angular_z = 0
                self.docking = None
                self.docking_mode_pub.publish(DockingMode(state=DockingMode.STATUS_COMPLETE))
                exit()
            self.cmd_vel_pub.publish(Twist(linear=Vector3(x=self.linear_x, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-self.angular_z)))
            

def main(args=None):
    rclpy.init(args=args)
    marker_docking = MarkerDocking()

    rclpy.spin(marker_docking)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
