#!/usr/bin/env python

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

import math

def rotate(origin, point, angle):
    """
    Source: https://stackoverflow.com/questions/34372480/rotate-point-about-another-point-in-degrees-python
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy


def euler_to_quaternion(roll, pitch, yaw):
    """
    Source: https://stackoverflow.com/questions/53033620/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
    Note: Order of inputs is edited from source.

    :return: list of quaternion values
    """
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)

    return [qx, qy, qz, qw]
class GripperClient(object):
    """
    Sourced and edited from demo.py in fetch_gazebo/fetch_gazebo_demo
    """
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
    """
    Sourced and edited from demo.py in fetch_gazebo/fetch_gazebo_demo
    """

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
    # Used to move robot along the x axis

    def __init__(self, pose=None):
        # Set flag, allowable error for controller, and create pub/sub for cmd_vel and odom data
        self.gotoFlag = False
        self.position_error = .1
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # Generate initial twist
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        self.twist = twist

    def odom_callback(self, data):
        # Callback to save the x position of the odometry data; skip if not needed
        if self.gotoFlag:
            self.x = data.pose.pose.position.x

    def goto(self, x_goal):
        # Simple controller to drive robot to desired x position
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
    # Collects one image from the head camera, and finds a grip for the handle,
    # then effectively turns off

    def __init__(self):
        # Set up flags and sub to the pointcloud
        rospy.sleep(.5)
        rospy.Subscriber('/head_camera/depth_registered/points', PointCloud2, self.pc2_callback)
        self.turnOffFlag = False
        self.waitFlag = True

    def pc2_callback(self, cloud):
        # Runs once when turned on based on the turnOffFlag
        # plots are created but not shown unless uncommented
        if not self.turnOffFlag:
            print("started")
            # Collect point cloud, convert, then reassign to correct x, y and z dimensions
            gen = pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z", "rgb"))
            cloud_list = list(gen)
            print("converted")
            self.turnOffFlag = True
            matrix = np.array(cloud_list)
            print(matrix.shape)
            y = -matrix[:, 0]
            z = -matrix[:, 1]
            x = matrix[:, 2]
            plot = plt.scatter(x, z, s=1)
            # plt.show()
            plot = plt.scatter(x, y, s=1)
            # plt.show()
            plot = plt.scatter(y, z, s=1)
            # plt.show()
            print(x.shape, z.shape)
            print("done")
            # Now, get the handle isolated and then find average location
            keepTrying = True # not used as a loop anymore, but just a single sequence
            x_range = .02
            z_floor = -0.8
            while keepTrying:
                # get the isolated location, by removing the floor, and then finding the nearest point in the x
                # direction; then gather all points within a certain depth based on x_range
                m_copy = np.array([x, y, z])
                print(m_copy.shape)
                m_copy = m_copy[:, z>(z_floor)]
                min_x = min(m_copy[0, :])
                m_copy = m_copy[:, m_copy[0]<(min_x+x_range)]
                print(m_copy.shape)
                print(m_copy)
                xn, yn, zn = m_copy[0, :], m_copy[1, :], m_copy[2, :]
                # Optional plots to view handle
                plot = plt.scatter(xn, zn)
                # plt.show()
                plot = plt.scatter(xn, yn)
                # plt.show()
                plot = plt.scatter(yn, zn)
                # plt.show()
                # Get Center for grip by taking avg position of points and save as self.gripLocation
                sum = np.average(m_copy, axis=1)
                print(sum)
                self.gripLocation = sum
                keepTrying = False
        self.waitFlag = False

class SensorClient2:
    def __init__(self):
        rospy.sleep(.5)
        self.firstPoint = None
        rospy.Subscriber('/head_camera/depth_registered/points', PointCloud2, self.pc2_callback2)
        self.turnOffFlag = False
        self.waitFlag = True

    def pc2_callback2(self, cloud):
        # Runs once when turned on based on the turnOffFlag
        # plots are created but not shown unless uncommented
        if not self.turnOffFlag:
            # Collect point cloud, convert, then reassign to correct x, y and z dimensions
            self.waitFlag = True
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
            # plot = plt.scatter(x, z, s=1)
            # # plt.show()
            # plot = plt.scatter(x, y, s=1)
            # # plt.show()
            # plot = plt.scatter(y, z, s=1)
            # # plt.xlim(-.1, .1)
            # # plt.ylim(2.6, 2.8)
            # # plt.rc('axes', axisbelow=True)
            # # plt.show()
            print(x.shape, z.shape)
            print("done")
            # Now, get the handle isolated and then find average location
            keepTrying = True
            min_x = min(x)
            x_range = .02
            z_floor = -0.8
            pointOne = [None, -.3, .1]
            pointTwo = [None, .3, .1]
            points = [pointOne, pointTwo]
            resultPoints = []
            windowSize = .03 / 2
            for p in points:
                # for both desired points, create a window around it, containing all points within a range,
                # and then just pick the first one, so that it is close enough to the desired point
                m_copy = np.array([x, y, z])
                print(m_copy.shape)
                m_copy = m_copy[:, m_copy[1]<(p[1]+windowSize)]
                m_copy = m_copy[:, m_copy[1]>(p[1]-windowSize)]
                m_copy = m_copy[:, m_copy[2]<(p[2]+windowSize)]
                m_copy = m_copy[:, m_copy[2]>(p[2]-windowSize)]
                print(m_copy.shape)
                resultPoints.append(m_copy[:, 0])
            # Calculate a line/vector between them and then save the line and first point
            self.line = resultPoints[1] - resultPoints[0]
            self.point = resultPoints[1]
        self.waitFlag = False


if __name__ == '__main__':
    rospy.init_node("hi")
    rospy.sleep(.5)
    mc = MoveClient()
    gac = GripperClient()
    gac.tuck()

    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm_with_torso", "base_link")

    print("Moving Now!")
    mc.goto(2.1)
    rospy.sleep(.1)
    while mc.gotoFlag:
        rospy.sleep(.1)
    sensor = SensorClient()
    rospy.sleep(.1)
    while sensor.waitFlag:
        rospy.sleep(.1)

    # lines 316-328 sourced from demo.py in fetch_gazebo
    # Define ground plane
    # This creates objects in the planning scene that mimic the ground
    # If these were not in place gripper could hit the ground
    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    # planning_scene.addCube("my_frontd", 2, 0.0, -1.2, -1.0)

    # This is the wrist link not the gripper itself
    gripper_frame = 'wrist_roll_link'
    # Position and rotation of two "wave end poses"
    pose = euler_to_quaternion(3.14/2, 0, 0)

    # Set gripper poses for the first grip sequence
    BIAS = np.array([.0, .03, 1.065])
    gripLocation = sensor.gripLocation + BIAS
    # print(gripLocation)
    gL = gripLocation
    print(gL)
    x, y, z, w = pose[0], pose[1], pose[2], pose[3]
    gripper_poses = [Pose(Point(gL[0]-0.2, gL[1]-0, gL[2]), #get into place
                          Quaternion(x, y, z, w)),
                     Pose(Point(gL[0] - 0.0, gL[1] - 0, gL[2]), #hover over, close grip
                          Quaternion(x, y, z, w)),
                     Pose(Point(gL[0] - 0.1, gL[1] - 0.03, gL[2]), #open slightly
                          Quaternion(x, y, z, w)),
                     Pose(Point(gL[0] - 0.0, gL[1] - 0, gL[2]), #close again
                          Quaternion(x, y, z, w))]#,
                     # Pose(Point(gL[0] - 0.1, gL[1] - 0.03, gL[2]), #open slightly
                     #      Quaternion(x, y, z, w)),
                     # Pose(Point(gL[0] - 0.0, gL[1] - 0, gL[2]), #close again, release
                     #      Quaternion(x, y, z, w))]
                     # Pose(Point(gL[0] - 0.1, gL[1] - 0, gL[2]), #move gripper away
                          # Quaternion(x, y, z, w))]]
    #
    # Construct a "pose_stamped" message as required by moveToPose
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'

    # print("CLOSE GRIPPER")
    # Open the gripper
    max_effort = 10.0
    grip_position = 0.07
    gac = GripperClient()
    gac.move_gripper(grip_position, max_effort)
    gac.tuck()
    rospy.sleep(1)

    # Initialize sensorClient2 object and get the first door position
    lineSensor = SensorClient2()
    while lineSensor.waitFlag:
        rospy.sleep(.1)
    firstLine = lineSensor.line
    firstPoint = lineSensor.point
    # Carry out first set of gripper poses
    i = 0
    for pose in gripper_poses:
        # pose = gripper_poses[0]

        gripper_pose_stamped.header.stamp = rospy.Time.now()
        gripper_pose_stamped.pose = pose
        move_group.moveToPose(gripper_pose_stamped, gripper_frame)
        result = move_group.get_move_action().get_result()
        rospy.sleep(.61)
        if i == 1:
            # Close grip here
            grip_position = 0.02
            gac.move_gripper(grip_position, max_effort)
            rospy.sleep(1)

        if i == 2:
            # get the second door position
            lineSensor.waitFlag = True
            lineSensor.turnOffFlag = False
            while lineSensor.waitFlag:
                rospy.sleep(.1)
            secondLine = lineSensor.line
            secondPoint = lineSensor.point
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

    # print(firstLine, secondLine, "xxxxxxxxxxxxxxxxxxx\n\n\n\n\n^Lines found\n\nxxxxxxxxxxxxxxxxxxxxxxxxxx\n", firstPoint, secondPoint)

    # Less robust but mroe generalized equation that calculates door axis
    # slope = firstLine[0]/firstLine[1] - secondLine[0]/secondLine[1] #x/y
    slope = firstLine[1]/firstLine[0] - secondLine[1]/secondLine[0] #y/x
    # uSlope = slope / slope[0]
    xDelta = firstPoint[0] - secondPoint[0]
    # yShift = xDelta * uSlope[1]
    yShift = xDelta * secondLine[1]/secondLine[0]
    uFirstLine = firstLine / firstLine[1]
    axisPoint = yShift * uFirstLine #+ firstPoint
    axisPoint = [secondPoint[0] + xDelta, secondPoint[1] + yShift]
    print(axisPoint)
    # Debugging statement:
    # print("slope:", slope,
    #       # "uSlope:", uSlope,
    #       "xDelta:", xDelta,
    #       "yShift", yShift,
    #       "uFirstLine", uFirstLine,
    #       "axisPoint", axisPoint)

    # Assumes first line has very little change in x between two points
    # xDelta = firstPoint[0] - secondPoint[0]


    # Opening door Sequence 2

    x, y, z = gL#[0], gl[1], gL[2]
    BIAS = [.03, .11]
    origin = [axisPoint[0] + BIAS[0], axisPoint[1] + BIAS[1]]
    # rospy.spin()
    # define step size, initial angle counter, and the last angle
    step = math.pi / 60
    angle = 0
    endAngle = math.pi / 2
    rospy.sleep(.5)
    #make it move back and move the arm at the same time
    #current position is 2.2, effectively 2.1
    start = 2 #starting position
    stepCount = 0
    while angle < endAngle:
        #Begin moving through angles
        stepCount += 1
        x, y = rotate(origin, (x, y), step)
        angle += step*0.3
        print(x, y, " for ", angle)
        orient = euler_to_quaternion(3.14 / 2, 0, angle)
        pose = Pose(Point(x, y, z),
                    Quaternion(orient[0], orient[1], orient[2], orient[3]))
        gripper_pose_stamped.header.stamp = rospy.Time.now()
        gripper_pose_stamped.pose = pose
        move_group.moveToPose(gripper_pose_stamped, gripper_frame)
        result = move_group.get_move_action().get_result()
	# sleep, just to give sim time to catch up
        rospy.sleep(.3)
        # If the step count reaches multiple of ten, move back to give more room for the door
        # and then re-grip the handle
        if stepCount % 10 == 0:
            #open grip
            grip_position = 0.07
            gac.move_gripper(grip_position, max_effort)
            rospy.sleep(.5)
            #move back in x position by .3
            start = start - .3
            mc.goto(start)
            #move gripper forward
            x += .16
            pose = Pose(Point(x, y, z),
                        Quaternion(orient[0], orient[1], orient[2], orient[3]))
            gripper_pose_stamped.header.stamp = rospy.Time.now()
            gripper_pose_stamped.pose = pose
            move_group.moveToPose(gripper_pose_stamped, gripper_frame)
            result = move_group.get_move_action().get_result()
            x += .01
            pose = Pose(Point(x, y, z),
                        Quaternion(orient[0], orient[1], orient[2], orient[3]))
            gripper_pose_stamped.header.stamp = rospy.Time.now()
            gripper_pose_stamped.pose = pose
            move_group.moveToPose(gripper_pose_stamped, gripper_frame)
            result = move_group.get_move_action().get_result()
            rospy.sleep(1)
            origin[0] += .3
            #close grip again
            grip_position = 0.02
            gac.move_gripper(grip_position, max_effort)
            rospy.sleep(1)

    # end with the door nearly all the way open

