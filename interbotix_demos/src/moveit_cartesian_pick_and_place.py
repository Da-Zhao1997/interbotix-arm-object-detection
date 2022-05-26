#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy

class MoveItCartesianDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_cartesian_demo')
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('interbotix_arm')
        gripper = moveit_commander.MoveGroupCommander("interbotix_gripper")

        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame('world')
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
        
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        
        # Set the allowable error value of gripper movement
        gripper.set_goal_joint_tolerance(0.001)

        # Set the maximum speed and acceleration allow
        gripper.set_max_acceleration_scaling_factor(0.5)
        gripper.set_max_velocity_scaling_factor(0.5)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('Home')
        arm.go()
        rospy.sleep(1)
        gripper.set_named_target('Home')
        gripper.go()
        rospy.sleep(1)  
    
        # The robot arm moves above the target object and opens the gripper

        self.move_arm(arm,0.3,0.0,0.2)
        gripper.set_named_target('Open')
        gripper.go()
        rospy.sleep(1)
        # The robot arm moves to the grasping position
        self.move_arm(arm,0.35,0.0,0.2)

        # Close the gripper
        gripper.set_named_target('Closed')
        gripper.go()
        rospy.sleep(1)

        # The robot arm moves above the placement position
        self.move_arm(arm,0.3,0.3,0.2)

        # Open the gripper
        gripper.set_named_target('Open')
        gripper.go()
        rospy.sleep(1)

        # The robot arm moves back to the starting position
        arm.set_named_target('Home')
        arm.go()
        rospy.sleep(1)
        gripper.set_named_target('Home')
        gripper.go()
        rospy.sleep(1)


        # 控制机械臂先回到初始化位置
        arm.set_named_target('Sleep')
        arm.go()
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def move_arm(self,arm,x,y,z):
        end_effector_link = arm.get_end_effector_link()
        waypoints = []
        start_pose = arm.get_current_pose(end_effector_link).pose      
        waypoints.append(start_pose)
        wpose = deepcopy(start_pose)
        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = z
        waypoints.append(deepcopy(wpose))
        fraction = 0.0   #路径规划覆盖率
        maxtries = 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数

        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.01,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划
            # 尝试次数累加
            attempts += 1
            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")       
        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  
        rospy.sleep(1)

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
