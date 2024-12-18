import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose

class MoveitIkDemo:
    """
    逆向运动学
    使用：
    1. roslaunch ur_moveit_config demo.launch
    2. rosrun ur_program_move moveit_inverse_k.py
    结果：
    使用默认的KDL会出现Fail: ABORTED: TIMED_OUT错误
    换成TRAC-IK之后可以正确求解
    """

    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_ik_demo")
        r = rospy.Rate(1)

        # 初始化需要使用的move_group控制的机械臂的arm group
        arm = moveit_commander.MoveGroupCommander("ur_arm")

        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)

        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)

        # 设置位置（单位：m）和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.01)

        # 设置允许的最大加速度和速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        # 控制机械臂回到初始位置
        arm.set_named_target("home")
        arm.go()
        r.sleep()

        # 设置机械臂工作空间中的目标位姿
        # 位置使用三维坐标x，y，z坐标表示
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = 0.2593
        target_pose.pose.position.y = 0.0636
        target_pose.pose.position.z = 0.1787
        target_pose.pose.orientation.x = 0.70692
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 0.70729

        # 设置机械臂当前状态为运动初始状态
        # 亲测下面这行注释掉，程序也能正常运行
        arm.set_start_state_to_current_state()

        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)

        # 规划运动路径，得到一条轨迹
        plan_success, traj, planning_time, error_code = arm.plan()
        print("Plan result: ", plan_success)
        # 这里可以对轨迹进行处理，甚至是重新规划轨迹
        #
        #

        # 按照规划的运动路径控制机械臂运动
        if plan_success:
            arm.execute(traj)
            r.sleep()
        else:
            rospy.logerr("Failed to plan the trajectory!")

        # 控制机械臂回到初始位置
        arm.set_named_target('home')
        arm.go()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try: 
        MoveitIkDemo()
    except rospy.ROSInterruptException:
        pass


