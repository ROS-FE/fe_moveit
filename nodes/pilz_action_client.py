#! /usr/bin/env python

import rospy
import actionlib
import moveit_commander
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction

FOLLOW_JOINT_TRAJECTORY_AS = '/scaled_pos_joint_traj_controller/follow_joint_trajectory'

# Init the node
rospy.init_node('pilz_action_server_example')

# Create the robot class
robot = moveit_commander.RobotCommander()

# Create the Moveit classes
move_group_pilz = moveit_commander.MoveGroupCommander(
    'manipulator',
    robot_description='/robot_description')
move_group_pilz.set_planner_id('PTP')

# Create offsets for the motion
first_axis_offset = -0.2

# Create the action client to the follow_joint_trajectory action server
trajectory_client = actionlib.SimpleActionClient(
            FOLLOW_JOINT_TRAJECTORY_AS, FollowJointTrajectoryAction)
if not trajectory_client.wait_for_server():
    rospy.logerr("Could not connect to the follow_joint_trajectory action server! Exiting ...")
    exit()

# Move the robot with a trajectory created by PILZ
rospy.loginfo("Moving the robot with PILZ ...")
joint_goal_pilz = list(robot.get_current_state().joint_state.position)
joint_goal_pilz[0] = joint_goal_pilz[0] + first_axis_offset
trajectory_goal_pilz = FollowJointTrajectoryGoal()
computed_trajectory_pilz = move_group_pilz.plan(joint_goal_pilz)
trajectory_goal_pilz.trajectory = computed_trajectory_pilz.joint_trajectory
trajectory_client.send_goal_and_wait(trajectory_goal_pilz)



