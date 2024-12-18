import rospy, sys
import moveit_commander

class MoveitFkDemo:
    """
    正向运动学
    使用：
    1. roslaunch ur_moveit_config demo.launch
    2. rosrun ur_program_move moveit_forward_k.py
    """
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_fk_demo", anonymous=True)
        r = rospy.Rate(1)
        

        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander("ur_arm")

        # 设置机械臂运动的允许误差值
        arm.set_goal_joint_tolerance(0.001)

        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        r.sleep()

        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        joint_positions = [0.391410, -0.676384, -0.376217, 0.0, 1.052834, 0.454125]
        arm.set_joint_value_target(joint_positions)

        # 控制机械臂完成运动
        arm.go()
        r.sleep()

        arm.set_named_target('home')
        arm.go()
        r.sleep()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        MoveitFkDemo()
    except rospy.ROSInterruptException:
        pass