from copy import deepcopy
import rospy
import sys
import math
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np

class MoveitCricleDemo:
    """
    控制机械臂画圆
    使用：
    1. roslaunch ur_moveit_config demo.launch
    2 rosrun ur_program_move moveit_circle.py
    """
    def __init__(self):
        # 初始化MoveGroupCommander
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_circle_demo")
        r = rospy.Rate(1)

        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander("ur_arm")

        # 允许重新规划
        arm.allow_replanning(True)
        # 设置目标位置参考坐标系
        reference_frame = "base_link"
        arm.set_pose_reference_frame(reference_frame)
        # 设置一些参数
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)

        # 获取末端执行器的坐标系名称
        end_effector_link = arm.get_end_effector_link()
        # 控制机械臂回到初始位置
        arm.set_named_target("home")
        arm.go()
        r.sleep()

        # 设置目标位姿
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = 0.1
        target_pose.pose.position.y = 0.1
        target_pose.pose.position.z = 0.4
        target_pose.pose.orientation.x = 0
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 1

        # 设置机械臂运动的终端位姿
        arm.set_pose_target(target_pose, end_effector_link)
        arm.go()

        # 初始化路点列表
        waypoints = []

        # 将圆弧上的路径点加入列表
        waypoints.append(target_pose.pose)

        center_a = target_pose.pose.position.y
        center_b = target_pose.pose.position.z
        radius = 0.05

        for th in np.arange(0, 6.28, 0.02):
            target_pose.pose.position.y = center_a + radius * math.cos(th)
            target_pose.pose.position.z = center_b + radius * math.sin(th)
            gpose = deepcopy(target_pose)
            waypoints.append(gpose.pose)
        
        fraction = 0.0  # 路径规划覆盖率
        maxtries = 100  # 最大尝试规划次数
        attempts = 0  # 已尝试规划次数

        # 设置机械臂当前状态作为运动初始状态
        arm.set_start_state_to_current_state()

        while fraction < 1.0 and attempts < maxtries:
            attempts += 1
            (plan, fraction) = arm.compute_cartesian_path(waypoints, 0.01, True)
            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

        # 如果路径规划成功，则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm")
            arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(attempts) + " attempts.")
        
        r.sleep()

        # 控制机械臂回到初始位置
        arm.set_named_target("home")
        arm.go()
        r.sleep()

        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        MoveitCricleDemo()
    except rospy.ROSInterruptException:
        pass

