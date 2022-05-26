#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from math import radians
from  object_color_detector.srv import DetectObjectSrv
from copy import deepcopy
from std_msgs.msg import Float64

class MoveItPickAndPlace:
    def __init__(self):
        # Initialize the move_group A
        moveit_commander.roscpp_initialize(sys.argv)
        # Initialize the ROS no
        rospy.init_node('moveit_pick_and_place_demo')
        self.gripper_pub = rospy.Publisher('/wx250s/gripper/command',Float64,queue_size=10)
        # Initialize the arm group in the robot arm that needs to be controlled with the move grooup
        arm = moveit_commander.MoveGroupCommander("interbotix_arm")
         # Initialize the move group that control the gripper
        gripper = moveit_commander.MoveGroupCommander("interbotix_gripper")
        # Get the name of the end link
        # end_effector_link = arm.get_end_effector_link()
        camera_link = "wx250s/gripper_link"
        e_e_link = "wx250s/ee_arm_link"
        # Set the reference coordinate system to be used for the target position
        ref_frame = 'world'
        arm.set_pose_reference_frame(ref_frame)
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
        # 初始化场景对象
        scene = PlanningSceneInterface()
        rospy.sleep(1)
        # 移除场景中之前运行残留的物体
        scene.remove_attached_object(camera_link, 'tool')
        scene.remove_world_object('table') 
        scene.remove_world_object('rubbish_bin') 
        # 设置桌面的高度
        table_ground = 0.0
        # 设置table和tool的三维尺寸
        table_size = [2, 2, 0.1]
        tool_size = [0.01, 0.05, 0.05]
        rubbish_bin_size = [0.4, 0.1, 0.14]
        # 设置tool的位姿
        p = PoseStamped()
        p.header.frame_id = camera_link
        p.pose.position.x = 0.01
        p.pose.position.y = 0
        p.pose.position.z = 0.05
        p.pose.orientation.w = 1
        # 将tool附着到机器人的终端
        scene.attach_box(camera_link, 'tool', p, tool_size)
        # 将table加入场景当中
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'world'
        table_pose.pose.position.x = 0.0
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground - table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box('table', table_pose, table_size)
        # 将rubbish_bin加入场景当中
        rubbish_bin_pose = PoseStamped()
        rubbish_bin_pose.header.frame_id = 'world'
        rubbish_bin_pose.pose.position.x = 0.0
        rubbish_bin_pose.pose.position.y = 0.25
        rubbish_bin_pose.pose.position.z = rubbish_bin_size[2] / 2.0
        rubbish_bin_pose.pose.orientation.w = 1.0
        scene.add_box('rubbish_bin', rubbish_bin_pose, rubbish_bin_size)

        rospy.sleep(1)  
        # 更新当前的位姿
        arm.set_start_state_to_current_state()
        # arm moves to "Home"
        arm.set_named_target('Home')
        arm.go()
        rospy.sleep(0.5)
        first_run = 1
        # self.move_arm(arm,0.25,0.0,0.35,-0.005,0.719,0.004,0.695,ref_frame,e_e_link,2)
        # move to the calibration pose
        self.move_arm(arm,0.25,0.0,0.35,0.00, 2**.5/2., 0.00, 2**.5/2.,ref_frame,e_e_link,0.5)
        while 1:
            if first_run == 1:
                first_run = 0
            else:
                self.move_arm(arm,0.25,0.0,0.35,0.00, 2**.5/2., 0.00, 2**.5/2.,ref_frame,e_e_link,0.5)
                rospy.sleep(6)
            # call object detect service
            rospy.wait_for_service("/object_detect")
            try:
                detect_object_client = rospy.ServiceProxy("/object_detect", DetectObjectSrv)
                # response = detect_object_client(DectectObjectSrvRequest.RED_OBJECT)
                response = detect_object_client()
                # print(response)
                rospy.loginfo("Detect object over")
            except rospy.ServiceException:
                Pass

            if response.blueObjList == [] and response.redObjList == [] and response.greenObjList == []:
                rospy.loginfo("Detect nothing")
                break

            grasp_config = -0.032
            for obj in response.blueObjList:
                print(obj.position)
                [tar_x,tar_y] = self.p2p(obj.position.x,obj.position.y)
                tar_x += 0.015
                tar_y += 0.00
                print(tar_x,tar_y)
                self.move_arm_cart(arm,tar_x,tar_y,0.25,e_e_link,1)
                # self.move_arm(arm,tar_x,tar_y,0.15,0.00, 2**.5/2., 0.00, 2**.5/2.,ref_frame,e_e_link,0.5)
                # self.move_gripper(gripper,"Open",0.1)
                self.gripper_pub.publish(-0.02)# open the gripper
                self.move_arm_cart(arm,tar_x,tar_y,0.085,e_e_link,1)
                # self.move_gripper2(gripper,0.024,-0.024,0.1)
                self.gripper_pub.publish(grasp_config) #close the gripper
                rospy.sleep(0.3)
                self.move_arm_cart(arm,tar_x,tar_y,0.25,e_e_link,1)
                self.move_arm_cart(arm,-0.09,0.25,0.25,e_e_link,1)
                # self.move_arm(arm,-0.09,0.25,0.25,0.00, 2**.5/2., 0.00, 2**.5/2.,ref_frame,e_e_link,0.5)
                # self.move_gripper(gripper,"Open",0.1)
                self.gripper_pub.publish(-0.02)
                # self.move_arm(arm,tar_x,tar_y,0.0,0.00, 2**.5/2., 0.00, 2**.5/2.,ref_frame,e_e_link,2)
                # self.move_gripper(gripper,"Closed",0.5)
            
            for obj in response.redObjList:
                print(obj.position)
                [tar_x,tar_y] = self.p2p(obj.position.x,obj.position.y)
                tar_x += 0.015
                tar_y += 0.00
                print(tar_x,tar_y)
                # self.move_arm_cart(arm,0.09,0.09,0.25,e_e_link,0.5)
                self.move_arm_cart(arm,tar_x,tar_y,0.30,e_e_link,1)
                # self.move_arm(arm,tar_x,tar_y,0.15,0.00, 2**.5/2., 0.00, 2**.5/2.,ref_frame,e_e_link,1)
                # self.move_gripper(gripper,"Open",1)
                self.move_arm_cart(arm,tar_x,tar_y,0.085,e_e_link,1)
                # self.move_gripper2(gripper,0.024,-0.024,1)
                self.gripper_pub.publish(grasp_config) # close the gripper
                rospy.sleep(0.3)
                self.move_arm_cart(arm,tar_x,tar_y,0.25,e_e_link,1)
                self.move_arm_cart(arm,0,0.25,0.25,e_e_link,1) # move above the rubbish bin
                # self.move_arm(arm,0,0.25,0.25,0.00, 2**.5/2., 0.00, 2**.5/2.,ref_frame,e_e_link,1)
                # self.move_gripper(gripper,"Open",1)
                self.gripper_pub.publish(-0.02)
                # self.move_arm(arm,tar_x,tar_y,0.0,0.00, 2**.5/2., 0.00, 2**.5/2.,ref_frame,e_e_link,2)
                # self.move_gripper(gripper,"Closed",0.5)

            for obj in response.greenObjList:
                print(obj.position)
                [tar_x,tar_y] = self.p2p(obj.position.x,obj.position.y)
                tar_x += 0.015
                tar_y += 0.00
                print(tar_x,tar_y)
                # self.move_arm_cart(arm,0.09,0.09,0.25,e_e_link,0.5)
                # self.move_arm_cart(arm,tar_x,tar_y,0.25,e_e_link,1)
                self.move_arm(arm,tar_x,tar_y,0.15,0.00, 2**.5/2., 0.00, 2**.5/2.,ref_frame,e_e_link,1)
                # self.move_gripper(gripper,"Open",1)
                self.move_arm_cart(arm,tar_x,tar_y,0.085,e_e_link,1)
                # self.move_gripper2(gripper,0.024,-0.024,1)
                self.gripper_pub.publish(grasp_config)
                rospy.sleep(0.3)
                self.move_arm_cart(arm,tar_x,tar_y,0.25,e_e_link,1)
                self.move_arm_cart(arm,0.09,0.25,0.25,e_e_link,1)
                # self.move_arm(arm,0.09,0.25,0.25,0.00, 2**.5/2., 0.00, 2**.5/2.,ref_frame,e_e_link,1)
                # self.move_gripper(gripper,"Open",1)
                self.gripper_pub.publish(-0.02)
                # self.move_arm(arm,tar_x,tar_y,0.0,0.00, 2**.5/2., 0.00, 2**.5/2.,ref_frame,e_e_link,2)
                # self.move_gripper(gripper,"Closed",0.5)

        arm.set_named_target('Home')
        arm.go()
        self.gripper_pub.publish(-0.01)
        rospy.sleep(0.5)
        arm.set_named_target('Sleep')
        arm.go()
        rospy.sleep(0.5)

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def p2p(self,ax,ay):
        x = -0.00050 * ay + (0.32485)
        y = -0.00049 * ax + (0.12605)
        return x,y

    def move_arm(self,arm,x,y,z,ox,oy,oz,ow,reference_frame,end_effector_link,time):
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = ox
        target_pose.pose.orientation.y = oy
        target_pose.pose.orientation.z = oz
        target_pose.pose.orientation.w = ow
        arm.set_start_state_to_current_state()
        arm.set_pose_target(target_pose, end_effector_link)
        plan_success, traj, planning_time, error_code = arm.plan()
        arm.execute(traj)
        rospy.sleep(time)
    def move_arm_cart(self,arm,x,y,z,end_effector_link,time):
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
        rospy.sleep(time)
    def move_gripper(self,gripper,state,time):
        gripper.set_named_target(state)
        gripper.go()
        rospy.sleep(time)
    def move_gripper2(self,gripper,joint1,joint2,time):
        # 0.037 0.017
        joint_positions = [joint1, joint2]
        gripper.set_joint_value_target(joint_positions)
        gripper.go()
        rospy.sleep(time)

if __name__ == "__main__":
    MoveItPickAndPlace()