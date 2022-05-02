#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs
from math import pi


def arm_ready(group_name):
    move_group = moveit_commander.MoveGroupCommander(group_name)
    planning_frame = move_group.get_planning_frame()
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 1.0
    #pose_goal.orientation.y = 0.0
    #pose_goal.orientation.z = 0.5
    #pose_goal.orientation.w = 1.0
    pose_goal.position.x = -0.48
    pose_goal.position.y =  3.0
    pose_goal.position.z = 1.30
    move_group.set_pose_target(pose_goal)
    move_group.stop()
    plan = move_group.go(wait=True)
    move_group.clear_pose_targets()



    move_group = moveit_commander.MoveGroupCommander(group_name)
    planning_frame = move_group.get_planning_frame()
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 1.0
    #pose_goal.orientation.y = 0.0
    #pose_goal.orientation.z = 0.5
    #pose_goal.orientation.w = 1.0
    pose_goal.position.x = -0.48
    pose_goal.position.y =  3.2
    pose_goal.position.z = 1.30
    move_group.set_pose_target(pose_goal)
    move_group.stop()
    plan = move_group.go(wait=True)
    move_group.clear_pose_targets()
    pass


def simple_pick_place(group_name):
    # Write a pick and place pipeline and execute with MoveIt and Gazebo
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state(), "\n")

    pose_goal = geometry_msgs.msg.Pose()
    print("============ Printing robot state")
    print(pose_goal, "\n")

    pose_goal.orientation.w = -1.0
    pose_goal.position.x = -0.48
    pose_goal.position.y =  3.0
    pose_goal.position.z = 1.30
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()  # ensures that there is no residual movement
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()
    #pass


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('lecture5_pick_and_place')
    rospy.sleep(10)
    try:
        for i in range(10):
        #print("b"*1000)
            arm_ready("panda_arm_1")
        #simple_pick_place("arm1")
    except rospy.ROSInterruptException:
        print("oops something is wrong here :(")
        pass