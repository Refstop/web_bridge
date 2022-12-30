#!/usr/bin/env python
from __future__ import print_function
from tokenize import ContStr

import roslib
import rospy
import actionlib
import sys
import tf
import rospkg
import rosparam
import os
import glob
import numpy as np
import math
import copy
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion, PoseStamped
from waypoint_navigation.msg import Int16Array, DestinationArray
from std_msgs.msg import Int8, Bool
from indoor_2d_nav_aruco_detect.msg import FiducialTransformArray_i2n
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

class Node:
    def __init__(self, parent=None, id=None, pose=None):
        self.parent = parent
        self.id = id
        self.pose = pose

        self.f = 0
        self.g = 0
        self.h = 0

    def __eq__(self, other):
        return self.id == other.id

class waypoint_navigation:
    def __init__(self):
        self.dst_marker_id_sub = rospy.Subscriber('/indoor_2d_nav_gui/dst_marker_id', Int8, self.DstMarkerIDCallback)
        self.goback_sub = rospy.Subscriber('/marker_detection/goback', Int8, self.GoBackCallback)
        self.aruco_sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray_i2n, self.ArucoCallback)
        self.cur_pnt_pub = rospy.Publisher('/waypoint_navigation/cur_pnt', Int8, queue_size=10)
        self.isReached_pub = rospy.Publisher('/waypoint_navigation/isReached', Bool, queue_size=10)
        self.waypoints_pub = rospy.Publisher('/waypoint_navigation/waypoints', PoseArray, queue_size=10)
        self.isArrived_pub = rospy.Publisher('/waypoint_navigation/isArrived', Bool, queue_size=1)
        self.dst_info_pub = rospy.Publisher('/waypoint_navigation/dst_info', DestinationArray, queue_size=1)
        self.listener = tf.TransformListener()

        self.destinations = dict()
        self.graph = dict()
        self.dst_point = dict()
        self.cur_id = 100
        self.dst_id = 100
        # self.cur_id = 0
        # self.dst_id = 0
        self.aaa = False
        self.waypoints = []
        self.goback = False
        self.LoadWaypoints()

    ## 1. Load Waypoints from txt files
    def LoadWaypoints(self):
        dir = rospkg.RosPack().get_path('waypoint_navigation')
        fnames = sorted(glob.glob(dir+'/waypoints/*.txt')) # all .txt files in folder, in order of name
        for fn in fnames:
            fna = fn.split('/')[-1]
            point_old = Point()
            ## waypoint dictionary: (start_id, end_id): [waypoints, cost]
            self.destinations[(int(fna[0:2]), int(fna[2:4]))] = [[], 0]
            with open(fn, 'r') as f:
                while True:
                    line = f.readline()
                    if not line:
                        f.close()
                        break
                    pose = line.split(' ')
                    point = Point(float(pose[0]), float(pose[1]), float(pose[2]))
                    quaternion = Quaternion(float(pose[3]), float(pose[4]), float(pose[5]), float(pose[6]))
                    
                    self.destinations[(int(fna[0:2]), int(fna[2:4]))][0].append(Pose(point, quaternion))
                    if len(self.destinations[(int(fna[0:2]), int(fna[2:4]))][0]) > 1:
                        delta = [point.x - point_old.x, point.y - point_old.y]
                        self.destinations[(int(fna[0:2]), int(fna[2:4]))][1] += math.sqrt(float(delta[0])**2 + float(delta[1])**2)
                    point_old = point

            ## graph dictionary for Astar: id: [id, id, id, ...] - edit
            if not self.graph.get(int(fna[0:2]), False):
                self.graph[int(fna[0:2])] = [int(fna[2:4])]
            else:
                self.graph[int(fna[0:2])].append(int(fna[2:4]))
            
            if not self.graph.get(int(fna[2:4]), False):
                self.graph[int(fna[2:4])] = [int(fna[0:2])]
            else:
                self.graph[int(fna[2:4])].append(int(fna[0:2]))

            ## dst points dictionary: id: [name, pose]
            dst_startend = fna[5:-4].split('_')
            if not self.dst_point.get(int(fna[0:2]), False):
                self.dst_point[int(fna[0:2])] = [dst_startend[0], self.destinations[(int(fna[0:2]), int(fna[2:4]))][0][0]]
            if not self.dst_point.get(int(fna[2:4]), False):
                self.dst_point[int(fna[2:4])] = [dst_startend[1], self.destinations[(int(fna[0:2]), int(fna[2:4]))][0][-1]]

        # print(self.destinations)
        # print(self.graph)
        # print(self.dst_point)
    
    ## 2. When this node subscribe id of destination, get current point by camera or source method
    def GetCurrentInfo(self, cur_id, dst_id):
        self.waypoints = []
        if cur_id == 100:
            # trans = []; rot = []
            # while not trans:
            #     try:
            #         (trans,rot) = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #         continue
            # cur_pose = Pose(Point(trans[0], trans[1], trans[2]), Quaternion(rot[0], rot[1], rot[2], rot[3]))
            ## test code
            # cur_pose = Pose(Point(6.82721996307, -9.60855102539, 0.0), Quaternion(0.0, 0.0, -0.0410141572078, 0.999158565448)) # test
            cur_pose = Pose(Point(-4.09759206893, -5.77592564001, 0.0), Quaternion(0.0, 0.0, 0.00363965686952, -0.999993376427)) # 0
            # cur_pose = Pose(Point(5.64700821587, -0.243424601964, 0.0), Quaternion(0.0, 0.0, 0.687493191914, -0.726190822768)) # 1
            # cur_pose = Pose(Point(23.3872986508, -3.00232416674, 0.0), Quaternion(0.0, 0.0, -0.999852106887, -0.0171978008642)) # 2
            # cur_pose = Pose(Point(16.6606492402, -9.09954141537, 0.0), Quaternion(0.0, 0.0, -0.697901507933, -0.716193748384)) # 3
            closest_dist = closest_dist_second = float('inf')
            closest_id = 100
            for id, val in self.dst_point.items():
                if self.EuclideanDistance(val[1], cur_pose) < closest_dist:
                    closest_dist_second = closest_dist
                    closest_dist = self.EuclideanDistance(val[1], cur_pose)
                    closest_id_second = closest_id
                    closest_id = id
                elif closest_dist < self.EuclideanDistance(val[1], cur_pose) < closest_dist_second:
                    closest_dist_second = self.EuclideanDistance(val[1], cur_pose)
                    closest_id_second = id
            
            if dst_id == closest_id:
                closest_id = closest_id_second
        else:
            closest_id = cur_id
            cur_pose = None
        # print(closest_id, cur_pose)
        return closest_id, cur_pose

    ## 3. Concatenate waypoints in reference to start_id & end_id(forward, backward)
    ## Use A* algorithm
    def WaypointConcatenate(self, cur_id, dst_id):
        ## cur: self.cur_id
        ## dst: self.dst_id
        if cur_id == 100 or dst_id == 100: return
        waypoints = []
        reverse = False
        if cur_id > dst_id:
            temp_id = cur_id
            cur_id = dst_id
            dst_id = temp_id
            reverse = True
        # always forward(ascending order)
        astar_result = self.AStar(cur_id, dst_id, self.graph, self.dst_point)
        # astar_result = [1, 3]
        if reverse:
            print(list(reversed(astar_result)))
        else:
            print(astar_result)

        if astar_result == None:
            print('Cannot generate waypoint array.')
            exit()
        
        ## edit
        if len(astar_result) == 1:
            waypoints.append(self.dst_point[astar_result[0]][1])
        elif len(astar_result) == 2:
            waypoints += self.destinations[(astar_result[0], astar_result[1])][0]
        else:
            for i in range(len(astar_result)-1):
                # concat first two points(ex: [0,1] of [0,1,3,4,5])
                if (astar_result[i], astar_result[i+1]) == (astar_result[0], astar_result[1]):
                    print(self.destinations[(astar_result[i], astar_result[i+1])][0][:-1])
                    waypoints += self.destinations[(astar_result[i], astar_result[i+1])][0][:-1]
                # concat last two points(ex: [4,5] of [0,1,3,4,5])
                elif (astar_result[i], astar_result[i+1]) == (astar_result[-2], astar_result[-1]):
                    print(self.destinations[(astar_result[i], astar_result[i+1])][0][:-1])
                    waypoints += self.destinations[(astar_result[i], astar_result[i+1])][0][1:]
                else:
                    waypoints += self.destinations[(astar_result[i], astar_result[i+1])][0][1:-1]
        
        if reverse:
            waypoints = self.ReverseWaypoints(waypoints)
        
        # print(waypoints)
        return waypoints
    
    ## 4. Adjust Waypoints to current pose
    def WaypointDetailSetting(self, waypoints, cur_pose):
        if cur_pose == None: return waypoints
        dist = float('inf')
        for i in range(len(waypoints)):
            if self.EuclideanDistance(waypoints[i], cur_pose) < dist:
                dist = self.EuclideanDistance(waypoints[i], cur_pose)
                cur_index = i
        # self.cur_id = 100; self.dst_id = 100
        return waypoints[cur_index:]

    ## 5. Drive Waypoints Sequentially
    def SequentialDrive(self, waypoints):
        move_waypoint = rospy.get_param('/move_waypoint', False)
        ## reset cur_id, dst_id
        self.cur_id = 100
        self.dst_id = 100
        for i in range(len(waypoints)):
            ## Emit first waypoint when state isn't 'goback'
            ## if use move_waypoint comment 2 lines below
            if not self.goback and waypoints[i] == waypoints[0] and len(waypoints) != 1 and not move_waypoint:
                continue
            self.MoveToGoal(waypoints[i], i)
        if not self.goback:
            self.isArrived_pub.publish(Bool(True))
        else:
            self.goback = False

    def WaypointNavigation(self):
        self.cur_id, cur_pose = self.GetCurrentInfo(self.cur_id, self.dst_id)
        # print("self.cur_id, cur_pose:", self.cur_id, cur_pose)
        waypoints = self.WaypointConcatenate(self.cur_id, self.dst_id)
        # print("waypoints:", waypoints)
        self.waypoints = self.WaypointDetailSetting(waypoints, cur_pose)
        # print("self.waypoints:", self.waypoints)
        self.SequentialDrive(self.waypoints)

    def FBCallback(self, feedback):
        os.system('rosservice call /move_base/clear_costmaps')
    
    def DstMarkerIDCallback(self, data):
        self.dst_id = data.data
        self.WaypointNavigation()

    def GoBackCallback(self, data):
        self.goback = True
        self.dst_id = data.data
        self.WaypointNavigation()

    def ArucoCallback(self, data):
        if data.detected_count > 1:
            print('Please detect just one marker.')
            return
        self.cur_id = data.transforms[0].fiducial_id

    def AStar(self, start_id, end_id, graph, dst_point):
        # init startNode and endNode
        startNode = Node(None, start_id, dst_point[start_id][1])
        endNode = Node(None, end_id, dst_point[end_id][1])

        # init openlist, closedlist
        openList = []
        closedList = []

        # Add startNode openList
        openList.append(startNode)
        while openList:
            currentNode = openList[0]
            currentIdx = 0
            for index, item in enumerate(openList):
                if item.f < currentNode.f:
                    currentNode = item
                    currentIdx = index

            openList.pop(currentIdx)
            closedList.append(currentNode)

            if currentNode == endNode:
                path = []
                current = currentNode
                while current is not None:
                    path.append(current.id)
                    current = current.parent
                return path[::-1]

            children = []
            if graph.get(currentNode.id, False):
                for new_id in graph[currentNode.id]:
                    newNode = Node(currentNode, new_id, dst_point[new_id][1])
                    children.append(newNode)

            for child in children:
                if child in closedList:
                    continue
                ## g cost: robot driving distance
                if currentNode.id < child.id:
                    child.g = currentNode.g + self.destinations[(currentNode.id, child.id)][1]
                else:
                    child.g = currentNode.g + self.destinations[(child.id, currentNode.id)][1]
                ## Heuristic: Euclidean Distance
                child.h = self.EuclideanDistance(child.pose, currentNode.pose)
                child.f = child.g + child.h

                if len([openNode for openNode in openList if child == openNode and child.g > openNode.g]) > 0:
                    continue
                    
                openList.append(child)
    
    def EuclideanDistance(self, p1, p2):
        return math.sqrt((p1.position.x - p2.position.x)**2 + (p1.position.y - p2.position.y)**2)

    def FiletypeCheck(self, fname): ## add later
        if fname != '.txt':
            print('File type Error')
            return False
        else:
            return True

    def ReverseWaypoints(self, waypoints):
        r_waypoints = copy.deepcopy(list(reversed(waypoints))) # ???????
        for wp in r_waypoints[1:-1]:
            q = quaternion_multiply([wp.orientation.x, wp.orientation.y, wp.orientation.z, wp.orientation.w], quaternion_from_euler(0, 0, math.pi))
            r_waypoints[r_waypoints.index(wp)].orientation = Quaternion(q[0], q[1], q[2], q[3])
        return r_waypoints

    def MoveToGoal(self, dst, wp_num):
        ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            print('Waiting for the move_base action server to come up')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = dst
  
        print('Sending goal location: ' + str(wp_num))
        ac.send_goal(goal, feedback_cb=self.FBCallback)
        ac.wait_for_result(rospy.Duration(6000))

        self.isReached_pub.publish(Bool(ac.get_state() ==  GoalStatus.SUCCEEDED))
    
    def pose_viz(self):
        print('Publishing Waypoints')
        rate = rospy.Rate(10.0)
        posearray_msg = PoseArray()
        while not rospy.is_shutdown():
            try:
                dst_ids = []
                dst_names = []
                posearray_msg.header.frame_id = '/map'
                posearray_msg.header.stamp = rospy.Time.now()
                posearray_msg.poses = self.waypoints
                self.waypoints_pub.publish(posearray_msg)
                for key in self.dst_point:
                    dst_ids.append(key)
                    dst_names.append(self.dst_point[key][0])
                # dst_ids = [0,1,3,2]
                self.dst_info_pub.publish(DestinationArray(dst_ids, dst_names))
                rate.sleep()
            except:
                continue

if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_navigation', anonymous=False)
        map_nav = waypoint_navigation()
        map_nav.pose_viz()
        rospy.spin()

    except rospy.ROSInterruptException:
        print('waypoint_navigation node terminated.')