import rospy
import sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy

class MoveitCartesianDemo:
    """
    笛卡尔空间运动
    使用：
    1. roslaunch ur_moveit_config demo.launch
    2.1 rosrun ur_program_move moveit_cartesian.py _cartesian:=True  (走直线)
    2.2 rosrun ur_program_move moveit_cartesian.py _cartesian:=True  (自由曲线)
    """
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_cartesian")
        r = rospy.Rate(1)

        # 是否需要使用笛卡尔空间的运动规划
        cartesian = rospy.get_param("~cartesian", True)
        print("cartesian:", cartesian)

        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander("ur_arm")

        # 允许在规划失败的情况下重新规划
        arm.allow_replanning(True)

        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame('base_link')

        # 设置位置（单位：米）和姿态（单位：弧度）的允许误差
        # 设置允许的最大加速度和速度
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

        # 控制机械臂回到初始化位置
        arm.set_named_target("home")
        arm.go()
        r.sleep()

        # 获取当前位姿数据作为机械臂起始运动位姿
        start_pose = arm.get_current_pose(end_effector_link).pose

        # 初始化路径点列表
        waypoints = []

        # 将初始位姿加入路点列表
        if cartesian:
            waypoints.append(start_pose)
        
        # 设置路径点数据，并加入路径点列表
        wpose = deepcopy(start_pose)
        wpose.position.z -= 0.2

        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            arm.set_pose_target(wpose)
            arm.go()
            r.sleep()
        
        wpose.position.x += 0.15

        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            arm.set_pose_target(wpose)
            arm.go()
            rospy.sleep(1)
        
        wpose.position.y += 0.1

        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            arm.set_pose_target(wpose)
            arm.go()
            rospy.sleep(1)

        wpose.position.x -= 0.15
        wpose.position.y -= 0.1

        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            arm.set_pose_target(wpose)
            arm.go()
            rospy.sleep(1)

        if cartesian:
            fraction = 0.0  # 路径规划覆盖率
            maxtries = 100  # 最大尝试规划次数
            attempts = 0    # 已经尝试规划次数

            # 设置机械臂当前的状态作为起始状态
            arm.set_start_state_to_current_state()

            # 尝试规划一条笛卡尔空间下的路径，依次通过所有路径点
            while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = arm.compute_cartesian_path(
                    waypoints, # 路径点列表
                    0.01,  #eef_step 终端步进值
                    # 0.0,  # jump_threshold 跳跃阈值
                    True  # avoid_collisions 避障规划
                )
            attempts += 1

            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
        
            # 如果路径规划成功，即路径覆盖率100%，则开始控制机械臂运动
            if fraction == 1.0:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                arm.execute(plan)
                rospy.loginfo("Path execution complete.")
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after" + str(maxtries) + " attempts.")
            
            r.sleep()

        # 控制机械臂回到初始位置
        arm.set_named_target("home")
        arm.go()
        r.sleep()

        # 关闭Moveit
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        MoveitCartesianDemo()
    except rospy.ROSInterruptException:
        pass



