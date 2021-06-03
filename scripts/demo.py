#!/usr/bin/env python

# wave.py: "Wave" the fetch gripper
import rospy
import math
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
import numpy as np
# from matplotlib.pyplot import scatter
import matplotlib.pyplot as plt

import time
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import trajectory_msgs.msg


import copy
import actionlib
import rospy

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.msg import OccupancyGrid, Odometry
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
# Note: fetch_moveit_config move_group.launch must be running
# Safety!: Do NOT run this script near people or objects.
# Safety!: There is NO perception.
#          The ONLY objects the collision detection software is aware
#          of are itself & the floor.

def euler_to_quaternion(roll, pitch, yaw): #taken from stack overflow
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)

    return [qx, qy, qz, qw]
class GripperClient(object):
    def __init__(self):

        rospy.loginfo("Waiting for gripper_controller...")
        self.gripper_client = actionlib.SimpleActionClient(
            "gripper_controller/gripper_action", GripperCommandAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("...connected.")
        self.move_group = MoveGroupInterface("arm", "base_link")

    def move_gripper(self, gripper_x, max_effort, timeout=5.0):

        gripper_goal = GripperCommandGoal()
        gripper_goal.command.max_effort = max_effort
        gripper_goal.command.position = gripper_x

        self.gripper_client.send_goal(gripper_goal)
        result = self.gripper_client.wait_for_result(rospy.Duration(timeout))

        return result

    def tuck(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

class MoveClient:
    def __init__(self, pose=None):
        self.gotoFlag = False
        self.position_error = .1
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        self.twist = twist

    def odom_callback(self, data):
        if self.gotoFlag:
            self.x = data.pose.pose.position.x

    def goto(self, x_goal):
        self.gotoFlag = True
        rospy.sleep(1)
        while self.gotoFlag:
            print("X position: ", self.x)
            error = x_goal - self.x
            if self.x  - x_goal < -self.position_error:
                self.twist.linear.x = error
                self.pub.publish(self.twist)
            elif self.x  - x_goal > self.position_error:
                self.twist.linear.x = error
                self.pub.publish(self.twist)
            else:
                self.gotoFlag = False
        self.twist.linear.x = 0
        self.pub.publish(self.twist)
        print("X position: ", self.x)


class SensorClient:
    def __init__(self):
        rospy.sleep(.5)
        rospy.Subscriber('/head_camera/depth_registered/points', PointCloud2, self.pc2_callback)
        self.turnOffFlag = False
        self.waitFlag = True

    def pc2_callback(self, cloud):
        if not self.turnOffFlag:
            print("started")
            gen = pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z", "rgb"))
            cloud_list = list(gen)
            print("converted")
            # for g in cloud_list:
            #     print(g)
            self.turnOffFlag = True
            matrix = np.array(cloud_list)
            print(matrix.shape)
            y = -matrix[:, 0]
            z = -matrix[:, 1]
            x = matrix[:, 2]
            plot = plt.scatter(x, z, s=1)
            #plt.show()
            plot = plt.scatter(x, y, s=1)
            #plt.show()
            plot = plt.scatter(y, z, s=1)
            # plt.xlim(-.1, .1)
            # plt.ylim(2.6, 2.8)
            # plt.rc('axes', axisbelow=True)
            #plt.show()
            print(x.shape, z.shape)
            print("done")
            # Now, get the handle isolated and then find average location
            keepTrying = True
            min_x = min(x)
            x_range = .02
            z_floor = -0.8
            while keepTrying:
                m_copy = np.array([x, y, z])
                print(m_copy.shape)
                m_copy = m_copy[:, z>(z_floor)]
                # x_new = m_copy[0, :]
                min_x = min(m_copy[0, :])
                m_copy = m_copy[:, m_copy[0]<(min_x+x_range)]
                print(m_copy.shape)
                print(m_copy)
                xn, yn, zn = m_copy[0, :], m_copy[1, :], m_copy[2, :]
                plot = plt.scatter(xn, zn)
                # plt.show()
                plot = plt.scatter(xn, yn)
                # plt.show()
                plot = plt.scatter(yn, zn)
                # plt.show()
                # Get Center for grip
                sum = np.average(m_copy, axis=1)
                print(sum)
                self.gripLocation = sum
                keepTrying = False
        self.waitFlag = False



if __name__ == '__main__':
    rospy.init_node("hi")
    rospy.sleep(5)

    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm_with_torso", "base_link")

    # sensor = SensorClient()
    # rospy.spin()

    mc = MoveClient()
    print("Moving Now!/n XXXXXXXXXXXXXX/n XXXXXXXXXXXXXX/n/n/n/n XXXXXXXXXXXX")
    mc.goto(1.5)
    mc.goto(2.3)
    rospy.sleep(.1)
    while mc.gotoFlag:
        rospy.sleep(.1)

    sensor = SensorClient()
    rospy.sleep(.1)
    while sensor.waitFlag:
        rospy.sleep(.1)
    # rospy.spin()

    # twist = Twist()
    # twist.linear.x = 0.10;
    # # twist.linear.y = 0.0;
    # # twist.linear.z = 0.0
    # # twist.angular.x = 0.0;
    # # twist.angular.y = 0.0;
    # twist.angular.z = 0.0
    # # twist = twist
    # pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # while 1:
    #     pub.publish(twist)
    #"""
    # move_base = MoveBaseClient()
    # move_base.goto(2.1, 0, 0.0)
    # Define ground plane
    # This creates objects in the planning scene that mimic the ground
    # If these were not in place gripper could hit the ground
    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    # planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    # planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    # planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    # planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

    # This is the wrist link not the gripper itself
    gripper_frame = 'wrist_roll_link'
    # Position and rotation of two "wave end poses"
    pose = euler_to_quaternion(3.14/2, 0, 0)

    BIAS = np.array([.0, .03, 1.065])
    gripLocation = sensor.gripLocation + BIAS
    print(gripLocation)
    gL = gripLocation
    x, y, z, w = pose[0], pose[1], pose[2], pose[3]
    gripper_poses = [Pose(Point(gL[0]-0.2, gL[1]-0, gL[2]), #get into place
                          Quaternion(x, y, z, w)),
                     Pose(Point(gL[0] - 0.0, gL[1] - 0, gL[2]), #hover over, close grip
                          Quaternion(x, y, z, w)),
                     Pose(Point(gL[0] - 0.1, gL[1] - 0.03, gL[2]), #open slightly
                          Quaternion(x, y, z, w)),
                     Pose(Point(gL[0] - 0.0, gL[1] - 0, gL[2]), #close again
                          Quaternion(x, y, z, w)),
                     Pose(Point(gL[0] - 0.1, gL[1] - 0.03, gL[2]), #open slightly
                          Quaternion(x, y, z, w)),
                     Pose(Point(gL[0] - 0.0, gL[1] - 0, gL[2]), #close again, release
                          Quaternion(x, y, z, w)),
                     Pose(Point(gL[0] - 0.2, gL[1] - 0, gL[2]), #move gripper away
                          Quaternion(x, y, z, w))]

    # Construct a "pose_stamped" message as required by moveToPose
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'

    print("CLOSE GRIPPER")
    max_effort = 10.0
    grip_position = 0.07
    gac = GripperClient()
    gac.move_gripper(grip_position, max_effort)
    gac.tuck()
    rospy.sleep(1)

    # if/while not rospy.is_shutdown():
    # if not rospy.is_shutdown():
    i = 0
    for pose in gripper_poses:
        # pose = gripper_poses[0]
        gripper_pose_stamped.header.stamp = rospy.Time.now()
        gripper_pose_stamped.pose = pose
        move_group.moveToPose(gripper_pose_stamped, gripper_frame)
        result = move_group.get_move_action().get_result()
        rospy.sleep(1)
        if i == 1:
            grip_position = 0.02
            gac.move_gripper(grip_position, max_effort)
            rospy.sleep(1)

        if i == 5:
            grip_position = 0.07
            gac.move_gripper(grip_position, max_effort)
            rospy.sleep(1)
        i = i + 1

        if result:
            # Checking the MoveItErrorCode
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Hello there!")
            else:
                # If you get to this point please search for:
                # moveit_msgs/MoveItErrorCodes.msg
                rospy.logerr("Arm goal in state: %s",
                             move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt! failure no result returned.")

    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    # move_group.get_move_action().cancel_all_goals()

    print("CLOSE GRIPPER")
    max_effort = 10.0
    grip_position = 0.05
    gac = GripperClient()
    gac.move_gripper(grip_position, max_effort)
    gac.tuck()


    # rospy.loginfo("Sleep 1 second")
    # rospy.sleep(1)
    # rospy.loginfo("Starting gripper movement")
    # gripper_joint_angle = [0.049-.01, 0.047-.01]
    # joint_name = ["l_gripper_finger_joint", "r_gripper_finger_joint"]
    # gripper_joint_angle = [0.6]
    # joint_name = ["l_gripper_finger_joint"]
    # move_group.moveToJointPosition(joint_name, gripper_joint_angle, wait=False)
    # move_group.get_move_action().wait_for_result()
    # result = move_group.get_move_action().get_result()
    # rospy.loginfo("Done!")
#"""
"""
import copy
import actionlib
import rospy

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

# Tools for grasping
class GraspingClient(object):

    def __init__(self):
        self.scene = PlanningSceneInterface("base_link")
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
        self.move_group = MoveGroupInterface("arm", "base_link")

        find_topic = "basic_grasping_perception/find_objects"
        rospy.loginfo("Waiting for %s..." % find_topic)
        self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)
        self.find_client.wait_for_server()

    def updateScene(self):
        # find objects
        goal = FindGraspableObjectsGoal()
        goal.plan_grasps = True
        self.find_client.send_goal(goal)
        self.find_client.wait_for_result(rospy.Duration(5.0))
        find_result = self.find_client.get_result()

        # remove previous objects
        for name in self.scene.getKnownCollisionObjects():
            self.scene.removeCollisionObject(name, False)
        for name in self.scene.getKnownAttachedObjects():
            self.scene.removeAttachedObject(name, False)
        self.scene.waitForSync()

        # insert objects to scene
        idx = -1
        for obj in find_result.objects:
            idx += 1
            obj.object.name = "object%d"%idx
            self.scene.addSolidPrimitive(obj.object.name,
                                         obj.object.primitives[0],
                                         obj.object.primitive_poses[0])#,
                                         # wait = False)

        for obj in find_result.support_surfaces:
            # extend surface to floor, and make wider since we have narrow field of view
            height = obj.primitive_poses[0].position.z
            obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],
                                            1.5,  # wider
                                            obj.primitives[0].dimensions[2] + height]
            obj.primitive_poses[0].position.z += -height/2.0

            # add to scene
            self.scene.addSolidPrimitive(obj.name,
                                         obj.primitives[0],
                                         obj.primitive_poses[0])#,
                                         # wait = False)

        self.scene.waitForSync()

        # store for grasping
        self.objects = find_result.objects
        self.surfaces = find_result.support_surfaces

    def getGraspableCube(self):
        graspable = None
        for obj in self.objects:
            # need grasps
            if len(obj.grasps) < 1:
                continue
            # check size
            if obj.object.primitives[0].dimensions[0] < 0.05 or \
               obj.object.primitives[0].dimensions[0] > 0.07 or \
               obj.object.primitives[0].dimensions[0] < 0.05 or \
               obj.object.primitives[0].dimensions[0] > 0.07 or \
               obj.object.primitives[0].dimensions[0] < 0.05 or \
               obj.object.primitives[0].dimensions[0] > 0.07:
                continue
            # has to be on table
            if obj.object.primitive_poses[0].position.z < 0.5:
                continue
            return obj.object, obj.grasps
        # nothing detected
        return None, None

    def getSupportSurface(self, name):
        for surface in self.support_surfaces:
            if surface.name == name:
                return surface
        return None

    def getPlaceLocation(self):
        pass

    def pick(self, block, grasps):
        success, pick_result = self.pickplace.pick_with_retry(block.name,
                                                              grasps,
                                                              support_name=block.support_surface,
                                                              scene=self.scene)
        self.pick_result = pick_result
        return success

    def place(self, block, pose_stamped):
        places = list()
        l = PlaceLocation()
        l.place_pose.pose = pose_stamped.pose
        l.place_pose.header.frame_id = pose_stamped.header.frame_id

        # copy the posture, approach and retreat from the grasp used
        l.post_place_posture = self.pick_result.grasp.pre_grasp_posture
        l.pre_place_approach = self.pick_result.grasp.pre_grasp_approach
        l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat
        places.append(copy.deepcopy(l))
        # create another several places, rotate each by 360/m degrees in yaw direction
        m = 16 # number of possible place poses
        pi = 3.141592653589
        for i in range(0, m-1):
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * pi / m)
            places.append(copy.deepcopy(l))

        success, place_result = self.pickplace.place_with_retry(block.name,
                                                                places,
                                                                scene=self.scene)
        return success

    def tuck(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    move_base = MoveBaseClient()
    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    head_action = PointHeadClient()
    grasping_client = GraspingClient()

    # Move the base to be in front of the table
    # Demonstrates the use of the navigation stack
    rospy.loginfo("Moving to table...")
    move_base.goto(3, 0, 0.0)
    rospy.loginfo("Done")
    # move_base.goto(2.250, 3.118, 0.0)
    # move_base.goto(2.750, 3.118, 0.0)

    # Raise the torso using just a controller
    # rospy.loginfo("Raising torso...")
    # torso_action.move_to([0.4, ])

    # Point the head at the cube we want to pick
    head_action.look_at(3.7, 3.18, 0.0, "map")

    # Get block to pick
    while not rospy.is_shutdown():
        rospy.loginfo("Picking object...")
        grasping_client.updateScene()
        cube, grasps = grasping_client.getGraspableCube()
        if cube == None:
            rospy.logwarn("Perception failed.")
            continue

        # Pick the block
        if grasping_client.pick(cube, grasps):
            break
        rospy.logwarn("Grasping failed.")

    # Tuck the arm
    grasping_client.tuck()

    # Lower torso
    rospy.loginfo("Lowering torso...")
    torso_action.move_to([0.0, ])

    # Move to second table
    rospy.loginfo("Moving to second table...")
    move_base.goto(-3.53, 3.75, 1.57)
    move_base.goto(-3.53, 4.15, 1.57)

    # Raise the torso using just a controller
    rospy.loginfo("Raising torso...")
    torso_action.move_to([0.4, ])

    # Place the block
    while not rospy.is_shutdown():
        rospy.loginfo("Placing object...")
        pose = PoseStamped()
        pose.pose = cube.primitive_poses[0]
        pose.pose.position.z += 0.05
        pose.header.frame_id = cube.header.frame_id
        if grasping_client.place(cube, pose):
            break
        rospy.logwarn("Placing failed.")

    # Tuck the arm, lower the torso
    grasping_client.tuck()
    torso_action.move_to([0.0, ])
"""
