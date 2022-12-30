#! /usr/bin/env python3
import glob
import math

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Pose, Point, PoseStamped, PoseArray
from marker_docking_interfaces.msg import DockingMode
from std_msgs.msg import String, Int8, Bool
from waypoint_web_client_interfaces.msg import Initialize, DisplayWaypoints
from rclpy.duration import Duration

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from waypoint_web_client import transformations

def EuclideanDistance(p1, p2):
    return math.sqrt((p1.position.x - p2.position.x)**2 + (p1.position.y - p2.position.y)**2)

class DestinationNode:
    def __init__(self, parent=None, id=None, pose=None):
        self.parent = parent
        self.id = id
        self.pose = pose

        self.f = 0
        self.g = 0
        self.h = 0

    def __eq__(self, other):
        return self.id == other.id

class WaypointAStar:
    def __init__(self):
        self.node = dict()
        self.edge = dict()
        self.graph = dict()

    def addEdge(self, id1, name1, id2, name2, waypoints, cost):
        ## waypoint dictionary: (start_id, end_id): [waypoints, cost]
        self.edge[(id1, id2)] = [waypoints, cost]

        ## graph dictionary for Astar: id: [id, id, id, ...]
        if not self.graph.get(id1, False):
            self.graph[id1] = [id2]
        else:
            self.graph[id1].append(id2)
        
        if not self.graph.get(id2, False):
            self.graph[id2] = [id1]
        else:
            self.graph[id2].append(id1)

        ## dst points dictionary: id: [name, pose]
        if not self.node.get(id1, False):
            self.node[id1] = [name1, self.edge[(id1, id2)][0][0]]
        if not self.node.get(id2, False):
            self.node[id2] = [name2, self.edge[(id1, id2)][0][-1]]

    def astar(self, start_id, end_id):
        # init startNode and endNode
        startNode = DestinationNode(None, start_id, self.node[start_id][1])
        endNode = DestinationNode(None, end_id, self.node[end_id][1])

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
            if self.graph.get(currentNode.id, False):
                for new_id in self.graph[currentNode.id]:
                    newNode = DestinationNode(currentNode, new_id, self.node[new_id][1])
                    children.append(newNode)

            for child in children:
                if child in closedList:
                    continue
                ## g cost: robot driving distance
                if currentNode.id < child.id:
                    child.g = currentNode.g + self.edge[(currentNode.id, child.id)][1]
                else:
                    child.g = currentNode.g + self.edge[(child.id, currentNode.id)][1]
                ## Heuristic: Euclidean Distance
                child.h = EuclideanDistance(child.pose, currentNode.pose)
                child.f = child.g + child.h

                if len([openNode for openNode in openList if child == openNode and child.g > openNode.g]) > 0:
                    continue
                    
                openList.append(child)

class WaypointWebClient(Node):
    def __init__(self):
        super().__init__('waypoint_web_client')
        # Launch the ROS 2 Navigation Stack
        # self.navigator = BasicNavigator()
        # self.navigator.waitUntilNav2Active()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.t = None
        # self.init = False
        self.t = 100
        self.init = True

        self.timer = self.create_timer(0.04, self.run)

        self.create_subscription(PoseArray, 'waypoints_web', self.waypoint_callback, 1)
        self.create_subscription(DockingMode, 'docking_mode', self.docking_mode_callback, 1)
        self.create_subscription(Int8, 'display_req', self.display_req_callback, 1)
        self.create_subscription(Bool, 'navstart', self.navstart_callback, 1)
        self.docking_mode_pub = self.create_publisher(DockingMode, 'docking_mode', 1)

        self.ui_init_pub = self.create_publisher(String, 'initialize', 1)
        self.display_pub = self.create_publisher(PoseArray, 'display_resp', 1)
        
        self.goal_poses = list()
        self.goal_poses_temp = list()
        self.goal_poses_last = None

        self.cur_id = 100
        self.dst_id = 100
        self.astar_handler = WaypointAStar()
        self.docking_complete = False

        serial_str = self.LoadWaypoints()
        self.ui_init_pub.publish(String(data=serial_str))
        self.get_logger().info("Start!")
    
    ## 1. Load Waypoints from txt files
    def LoadWaypoints(self):
        self.get_logger().info("File Loading...")
        dir = get_package_share_directory('waypoint_web_client')
        fnames = sorted(glob.glob(dir+'/waypoints/*.txt')) # all .txt files in folder, in order of name
        for fn in fnames:
            fna = fn.split('/')[-1][:-4].split('_')
            waypoint_old = Pose()
            waypoints, cost = [], 0
            with open(fn, 'r') as f:
                while True:
                    line = f.readline()
                    if not line:
                        f.close()
                        break
                    
                    pose = line.split(' ')
                    waypoint = Pose()
                    waypoint.position.x = float(pose[0])
                    waypoint.position.y = float(pose[1])
                    waypoint.position.z = float(pose[2])
                    waypoint.orientation.x = float(pose[3])
                    waypoint.orientation.y = float(pose[4])
                    waypoint.orientation.z = float(pose[5])
                    waypoint.orientation.w = float(pose[6])
                    waypoints.append(waypoint)
                    
                    if len(waypoints) > 1:
                        cost += EuclideanDistance(waypoint, waypoint_old)
                    waypoint_old = waypoint
            self.astar_handler.addEdge(int(fna[0]), fna[1], int(fna[2]), fna[3], waypoints, cost)

        serial_str = ""
        for marker_id, dst_info in self.astar_handler.node.items():
            serial_str += str(marker_id) + "_" + dst_info[0] + " "
        self.get_logger().info(serial_str)
        self.get_logger().info("Complete File Load.")
        return serial_str[:-1]
    ## 2. When this node subscribe id of destination, get current point by camera or source method
    def GetCurrentInfo(self, cur_id, dst_id):
        self.goal_poses = list()
        if cur_id == 100:
            # t = list()
            # while not t:
            #     try:
            #         t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #         continue
            # cur_pose = Pose()
            # cur_pose.position.x = t.transform.translation.x
            # cur_pose.position.y = t.transform.translation.y
            # cur_pose.position.z = 0.0

            # cur_pose.orientation = t.transform.rotation
            ## test code
            cur_pose = Pose()
            cur_pose.position.x = -1.254
            cur_pose.position.y = 0.866

            quat = transformations.quaternion_from_euler(0.0, 0.0, -2.042)
            cur_pose.orientation.x = quat[0]
            cur_pose.orientation.y = quat[1]
            cur_pose.orientation.z = quat[2]
            cur_pose.orientation.w = quat[3]
            
            closest_dist = closest_dist_second = float('inf')
            closest_id = 100
            for id, val in self.astar_handler.node.items():
                if EuclideanDistance(val[1], cur_pose) < closest_dist:
                    closest_dist_second = closest_dist
                    closest_dist = EuclideanDistance(val[1], cur_pose)
                    closest_id_second = closest_id
                    closest_id = id
                elif closest_dist < EuclideanDistance(val[1], cur_pose) < closest_dist_second:
                    closest_dist_second = EuclideanDistance(val[1], cur_pose)
                    closest_id_second = id
            
            if dst_id == closest_id:
                closest_id = closest_id_second
        else:
            closest_id = cur_id
            cur_pose = None
        return closest_id, cur_pose

    ## 3. Concatenate waypoints in reference to start_id & end_id(forward, backward)
    ## Use A* algorithm
    def WaypointConcatenate(self, cur_id, dst_id):
        ## cur: self.cur_id
        ## dst: self.dst_id
        if cur_id == 100 or dst_id == 100: return
        waypoints = []
        astar_result = self.astar_handler.astar(cur_id, dst_id)
        self.get_logger().info(str(astar_result))
        if astar_result == None:
            print('Cannot generate waypoint array.')
        
        if len(astar_result) == 1:
            waypoints.append(self.node[astar_result[0]][1])
        elif len(astar_result) == 2:
            waypoints += self.astar_handler.edge[(astar_result[0], astar_result[1])][0]
        else:
            for i in range(len(astar_result)-1):
                # concat first two points(ex: [0,1] of [0,1,3,4,5])
                if (astar_result[i], astar_result[i+1]) == (astar_result[0], astar_result[1]):
                    waypoints += self.astar_handler.edge[(astar_result[i], astar_result[i+1])][0][:-1]
                # concat last two points(ex: [4,5] of [0,1,3,4,5])
                elif (astar_result[i], astar_result[i+1]) == (astar_result[-2], astar_result[-1]):
                    waypoints += self.astar_handler.edge[(astar_result[i], astar_result[i+1])][0][1:]
                else:
                    waypoints += self.astar_handler.edge[(astar_result[i], astar_result[i+1])][0][1:-1]

        waypoints_stamped = []
        for wp in waypoints:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = self.navigator.get_clock().now().to_msg()
            ps.pose = wp
            waypoints_stamped.append(ps)
        return waypoints_stamped, waypoints

    def waypoint_callback(self, msg):
        if not self.goal_poses:
            for pose in msg.poses:
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                goal_pose.pose = pose
                self.goal_poses.append(goal_pose)

                self.get_logger().info(str(pose.orientation.z) + ' ' + str(pose.orientation.w))

    def docking_mode_callback(self, msg):
        if msg.state == DockingMode.STATUS_COMPLETE:
            self.docking_complete = True
            self.goal_poses.append(self.goal_poses_last)
    
    def display_req_callback(self, msg):
        self.dst_id = msg.data
        self.goal_poses_temp = list()
        [closest_id, cur_pose] = self.GetCurrentInfo(self.cur_id, self.dst_id)
        self.goal_poses_temp, waypoints = self.WaypointConcatenate(closest_id, self.dst_id)
        
        self.display_pub.publish(PoseArray(poses=waypoints))

    def navstart_callback(self, msg):
        if msg.data:
            self.goal_poses = self.goal_poses_temp

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

def main(args=None):
    rclpy.init(args=args)

    waypoint_web_client = WaypointWebClient()

    rclpy.spin(waypoint_web_client)

    waypoint_web_client.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()