import numpy as np
import matplotlib.pyplot as plt
import rospy, tf, math, universal_robot_kinematics as urk
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_srvs.srv import Empty
import math, copy, sys, rospy, moveit_commander, moveit_msgs.msg, geometry_msgs, numpy, time, std_msgs, actionlib_msgs
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

rospy.init_node('trajviz')
pub = rospy.Publisher('trajectory', Marker, queue_size=1)
br = tf.TransformBroadcaster()
def init_marker():
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD

    # marker scale
    marker.scale.x = 0.03
    marker.scale.y = 0.03
    marker.scale.z = 0.03

    # marker color
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    # marker orientaiton
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # marker position
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    # marker line points
    marker.points = []
    return marker
marker = init_marker()



class UR5():
    def __init__(self, clear_octomap):
        #initiating the commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")
        #self.group.allow_replanning(True)

        #functions to start displaying the trajectory in rviz
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.group.set_goal_tolerance(0.02) #setting joint tolerance
        self.group.set_max_acceleration_scaling_factor(0.05)
        self.group.set_max_velocity_scaling_factor(0.05)

        #publisher for markers
        self.pub = rospy.Publisher('trajectory', Marker, queue_size=1)

        #defining the table for collision avoidance
        self.box_pose = geometry_msgs.msg.PoseStamped()
        self.box_pose.header.frame_id = "base_link"
        self.box_pose.pose.orientation.z = 0.3826834
        self.box_pose.pose.orientation.w = 0.9238795
        self.box_pose.pose.position.x = -0.35
        self.box_pose.pose.position.z = -0.05
        self.scene.add_box("table", self.box_pose, size=(0.7, 0.7, 0.1))
        
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
        self.waypoints.append(self.group.get_current_pose().pose)

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
    
    def add_waypoint(self, x, y, z):
        self.point.x = x
        self.point.y = y
        self.point.z = z
        self.marker.points.append(self.point)
        self.wpose = geometry_msgs.msg.Pose()
        self.wpose.orientation.w = 1.0
        self.wpose.position.x = x
        self.wpose.position.y = y
        self.wpose.position.z = z
        self.waypoints.append(copy.deepcopy(self.wpose))

    def clear_waypoints(self):
        self.waypoints = []



    #Function to stop the robot.
    def stop():
        self.group.clear_pose_targets()
        self.group.stop()

rospy.wait_for_service('/clear_octomap')
clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
arm = UR5(clear_octomap)


arm.move_endpoint(((math.cos(0/100)*(math.sin(0/100)))/5)+0.5, (math.sin((0/100)+(math.pi/2)))/5, ((math.sin(0/100)*(math.sin(0/100)))/5)+ 0.2)

for e in range(0, int((math.pi)*100), int((math.pi*10))):
    x = math.cos(e/100)
    y = math.sin((e/100)+(math.pi/2))
    r = math.sin(e/100)

    for i in range(0, int((math.pi)*100), int((math.pi))):
        
        first = ((math.cos(i/100)*r)/5)+0.5
        if 'prev' in globals() and first != prev:

            point = Point()
            point.x = ((math.cos(i/100)*r)/5)+0.5
            point.y = y/5
            point.z = ((math.sin(i/100)*r)/5)+ 0.3
            
            arm.marker.points.append(point)
            arm.add_waypoint(point.x, point.y, point.z)
            marker.points.append(point)
        else:
            print("dumped", e, i)

        prev = ((math.cos(i/100)*r)/5)+0.5

#checks for dupes
for i in range(len(arm.waypoints)-1):
    if arm.waypoints[i+1] == arm.waypoints[i]:
        print('duplicate at ', i)
        print(arm.waypoints[i])
        print(arm.waypoints[i+1])

arm.group.set_max_acceleration_scaling_factor(0.2)
arm.group.set_max_velocity_scaling_factor(0.2)
arm.group.set_goal_tolerance(0.01)
arm.group.set_planning_time(20)

(plan, fraction) = arm.group.compute_cartesian_path(arm.waypoints, 0.01, 0.0, True)

arm.display_trajectory.trajectory_start = arm.robot.get_current_state()
arm.display_trajectory.trajectory.append(plan)

arm.group.execute(plan)



#while True:
pub.publish(marker)
moveit_commander.roscpp_shutdown()