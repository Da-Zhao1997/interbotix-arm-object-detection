#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose

Sleep_time1 = 1
Sleep_time2 = 2

class MoveItPickAndPlaceDemo:
    def __init__(self):

        # Initialize the move_group A
        moveit_commander.roscpp_initialize(sys.argv)
        # Initialize the ROS no
        rospy.init_node('moveit_pick_and_place_demo')
        # Initialize the arm group in the robot arm that needs to be controlled with the move grooup
        arm = moveit_commander.MoveGroupCommander("interbotix_arm")
         # Initialize the move group that control the gripper
        gripper = moveit_commander.MoveGroupCommander("interbotix_gripper")
        # Get the name of the end link
        end_effector_link = arm.get_end_effector_link()
        # Set the reference coordinate system to be used for the target position
        reference_frame = 'world'
        arm.set_pose_reference_frame(reference_frame)
        # Allow re-planning when  planning fails
        arm.allow_replanning(True)
        # Allowable error in setting position (in meters) and attitude (in radians)
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.01)
        # Set the maximum speed and acceleration allow
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        # Set the allowable error value of gripper movement
        gripper.set_goal_joint_tolerance(0.001)
        # Set the maximum speed and acceleration allow
        gripper.set_max_acceleration_scaling_factor(0.5)
        gripper.set_max_velocity_scaling_factor(0.5)

        # The robot moves to the starting position
        arm.set_named_target('Home')
        arm.go()
        rospy.sleep(1)
        gripper.set_named_target('Home')
        gripper.go()
        rospy.sleep(Sleep_time1)

        # The robot arm moves above the target object and opens the gripper
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.2
        target_pose.pose.orientation.w = 1.0
        arm.set_start_state_to_current_state()
        arm.set_pose_target(target_pose, end_effector_link)
        plan_success, traj, planning_time, error_code = arm.plan()
        # traj = arm.plan()
        arm.execute(traj)
        rospy.sleep(Sleep_time2)
        gripper.set_named_target('Open')
        gripper.go()
        rospy.sleep(Sleep_time1)

        # The robot arm moves to the grasping position
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.35
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.2
        target_pose.pose.orientation.w = 1.0
        arm.set_start_state_to_current_state()
        arm.set_pose_target(target_pose, end_effector_link)
        plan_success, traj, planning_time, error_code = arm.plan()
        # traj = arm.plan()
        arm.execute(traj)
        rospy.sleep(Sleep_time2)

        # Close the gripper
        gripper.set_named_target('Closed')
        gripper.go()
        rospy.sleep(Sleep_time1)

        # The robot arm moves above the placement position
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.3
        target_pose.pose.position.z = 0.2
        target_pose.pose.orientation.w = 1.0
        arm.set_start_state_to_current_state()
        arm.set_pose_target(target_pose, end_effector_link)
        plan_success, traj, planning_time, error_code = arm.plan()
        # traj = arm.plan()
        arm.execute(traj)
        rospy.sleep(Sleep_time2)

        # Open the gripper
        gripper.set_named_target('Open')
        gripper.go()
        rospy.sleep(Sleep_time1)

        # The robot arm moves back to the starting position
        arm.set_named_target('Home')
        arm.go()
        rospy.sleep(1)
        gripper.set_named_target('Home')
        gripper.go()
        rospy.sleep(1)
        arm.set_named_target('Sleep')
        arm.go()
        rospy.sleep(1)

        # Close and exit moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItPickAndPlaceDemo()

    
    


