#! /usr/bin/env python3
import math, copy, sys, rospy, moveit_commander, moveit_msgs.msg, geometry_msgs, numpy as np, time, std_msgs, actionlib_msgs
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class UR5():
 

    def __init__(self, clear_octomap):
        #initiating the commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.group.allow_replanning(True)
        
        #functions to start displaying the trajectory in rviz
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.group.set_goal_tolerance(0.02) #setting joint tolerance
        self.group.set_max_acceleration_scaling_factor(0.01)
        self.group.set_max_velocity_scaling_factor(0.01)
        
        
        #defining the table for collision avoidance
        self.box_pose = geometry_msgs.msg.PoseStamped()
        self.box_pose.header.frame_id = "base_link"
        self.box_pose.pose.orientation.z = 0.38
        #self.box_pose.pose.orientation.z = 0
        
        self.box_pose.pose.orientation.w = 0.9238795
        self.box_pose.pose.position.x = -0.35
        #self.box_pose.pose.position.x = -0.50
        self.box_pose.pose.position.z = -0.05
        self.scene.add_box("table", self.box_pose, size=(2, 2, 0.1))
        
        self.marker = Marker()
        self.marker.header.frame_id = "base_link"
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = self.marker.ADD

        # marker scale
        self.marker.scale.x = 0.03
        self.marker.scale.y = 0.03
        self.marker.scale.z = 0.03

        # marker color
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

        # marker orientaiton
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        # marker position
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.points = []


        #create pose & point
        self.point = Point()

        #waypoint list
        self.waypoints = []

    #simple function for moving the base of the arm
    def move_base(self, x):
        joint_pose_1 = self.group.get_current_joint_values()
        joint_pose_1[0] = x
        self.group.set_joint_value_target(joint_pose_1) #setting joint value targets
        self.display_trajectory.trajectory.append(joint_pose_1) #displaying trajectory in rviz
        self.display_trajectory_publisher.publish() #publishing trajectory
        self.group.go(wait=True)

    #function that takes a list of angles, the angles are for each joint of the robot starting from the base.
    def move_alljoints(self, stopdelay, list):
        joint_pose_1 = self.group.get_current_joint_values()
        for index, each in enumerate(list):
            joint_pose_1[index] = each
        time.sleep(stopdelay)
        self.group.set_joint_value_target(joint_pose_1) #setting joint value targets
        self.display_trajectory.trajectory.append(joint_pose_1) #displaying trajectory in rviz
        self.display_trajectory_publisher.publish() #publishing trajectory
        #clear_octomap()
        self.group.go(wait=True)


    #function to move the endpoint of the robot to a point x, y, z. 
    def move_endpoint(self, x, y, z):
        self.pose_target = geometry_msgs.msg.Pose()
        self.pose_target.orientation.w = 1.0
        self.pose_target.position.x = x
        self.pose_target.position.y = y
        self.pose_target.position.z = z
        self.group.set_pose_target(self.pose_target)
        self.display_trajectory.trajectory.append(self.pose_target) #displaying trajectory in rviz
        self.display_trajectory_publisher.publish() #publishing trajectory
        self.group.go(wait=True)
    
    def get_quaternion_from_euler(self, roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

    def add_waypoint(self, x, y, z, rx, ry, rz):
        q = self.get_quaternion_from_euler(rx, ry, rz)
        self.point.x = x
        self.point.y = y
        self.point.z = z
        self.marker.points.append(self.point)
        self.wpose = self.group.get_current_pose().pose
        self.wpose.position.x = x
        self.wpose.position.y = y
        self.wpose.position.z = z
        self.wpose.orientation.x = q[0]
        self.wpose.orientation.y = q[1]
        self.wpose.orientation.z = q[2]
        self.wpose.orientation.w = q[3]
        self.waypoints.append(copy.deepcopy(self.wpose))

    def clear_waypoints(self):
        self.waypoints = []



    #Function to stop the robot.
    def stop():
        self.group.clear_pose_targets()
        self.group.stop()





# Creating the node, subscribers and services
rospy.wait_for_service('/clear_octomap')
pub = rospy.Publisher('trajectory', Marker, queue_size=1)
clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
rospy.init_node('test_movement', anonymous=True)
# Creating arm object and defining variables.
arm = UR5(clear_octomap)






arm.clear_waypoints()

# The object position. The x-direction is diagonally away from the table through the robot.
# The first quadrant will be on the top left side of the robot, seen from the middle of the table. 
obj_x = -0.35 
obj_y = 0

# Offsets to mark the starting points of the 
offset_x = -0.35
offset_y = -0.1
# The starting height of the arc
height = 0.25

# Scaling up the size of the arc, note that the size of the arc is 0.1 if there is no scaling.
# Using a higher value than 2.5 on the x scale can lead to problems due to the physical limitations
# of the robot arm.
scale_x = 2.5
scale_y = 5
scale_z = 2.5


for e in range(0, int((math.pi)*100), int((math.pi*10))):
    y = math.sin((e/100)+(math.pi/2))
    r = math.sin(e/100)

    for i in range(0, int((math.pi)*100), int((math.pi))):
        point = Point()
        point.x = (((math.cos(i/100)*r)/10)*scale_x) + offset_x
        point.y = ((y/10)*scale_y) + offset_y
        point.z = (((math.sin(i/100)*r)/10)*scale_z) + height

        if 'prev' in globals() and point != prev:

            if point.x-obj_x > 0:
                yrh = math.sqrt(abs((point.z)**2 + (point.x-obj_x)**2))
                ry = math.asin((point.x-obj_x)/yrh) + math.pi
            else:
                yrh = math.sqrt(abs((point.z)**2 + (obj_x-point.x)**2))
                ry = -math.asin((obj_x-point.x)/yrh) + math.pi
            
            if point.y-obj_y > 0:
                xrh = math.sqrt(abs(point.z**2 + (point.y-obj_y)**2))
                rx = math.asin((point.y-obj_y)/xrh)
            else:
                xrh = math.sqrt(abs(point.z**2 + (obj_y-point.y)**2))
                rx =  -math.asin((obj_y-point.y)/xrh)


            arm.add_waypoint(point.x, point.y, point.z, rx, ry, 0)
            
            
            arm.marker.points.append(point)
        else:
            print("dumped", e, i)
            print()

        prev = point


arm.group.set_max_acceleration_scaling_factor(0.01)
arm.group.set_max_velocity_scaling_factor(0.01)
#arm.group.set_goal_tolerance(0.005)
arm.group.set_goal_orientation_tolerance(0.01)
#arm.group.set_goal_joint_tolerance(0.01)
arm.group.set_planning_time(100)
arm.group.set_num_planning_attempts(10000)
'''
point = Point()
point.x = -0.4
point.y = 0
point.z = 0.3
arm.add_waypoint(point.x, point.y, point.z, 0, 0, 0)
'''

(plan, fraction) = arm.group.compute_cartesian_path(arm.waypoints, 0.01, 0.0, True)
pub.publish(arm.marker)

arm.group.execute(plan)

moveit_commander.roscpp_shutdown()