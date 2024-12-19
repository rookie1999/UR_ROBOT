import rospy
import sys
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from math import radians
from copy import deepcopy

class MoveitAttachedObjectDemo:
    """
    添加附着物体与碰撞物体进行自主避障
    1. roslaunch ur_moveit_config demo.launch
    2 rosrun ur_program_move moveit_attached_object_demo.py
    注意：该demo下如果调整附着物体与末端坐标系的三维坐标关系，当该物体嵌入进末端，会直接显示碰撞产生
    """
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_avoid_collision")
        r = rospy.Rate(1)

        # 初始化场景对象
        scene = PlanningSceneInterface()
        r.sleep()

        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander("ur_arm")

        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

        # 设置误差值和最大加速度、速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)

        # 允许规划失败后重新规划
        arm.allow_replanning(True)
        arm.set_planning_time(10)

        # 控制机械臂回到初始化位置
        arm.set_named_target("home")
        arm.go()
        r.sleep()

        # 移除场景中之前运行的残留物体
        # scene.remove_attached_object(end_effector_link, "tool")
        scene.remove_world_object("table")

        # 设置桌面的高度
        table_height = 0.5
        # 设置table和tool的三维尺寸
        table_size = [0.1, 0.7, 0.01]
        tool_size = [0.2, 0.02, 0.02]

        # 设置tool位姿
        p = PoseStamped()
        p.header.frame_id = end_effector_link
        p.pose.position.x = tool_size[0] / 2. - 0.025
        p.pose.position.y = -0.015
        p.pose.position.z = 0.015
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 1

        # 将tool附着到机器人的终端
        scene.attach_box(end_effector_link, "tool", p, tool_size)

        # 将table加入到scene
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.position.x = -0.3
        table_pose.pose.position.y = -0.3
        table_pose.pose.position.z = table_height + table_size[2] / 2. + 0.1
        table_pose.pose.orientation.x = 0
        table_pose.pose.orientation.y = 0
        table_pose.pose.orientation.z = 0
        table_pose.pose.orientation.w = 1.
        scene.add_box("table", table_pose, table_size)

        # 更新当前位姿
        arm.set_start_state_to_current_state()

        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        joint_positions = [0.827228546495185, 0.29496592875743577, 1.1185644936946095, -0.7987583317769674, -0.18950024740190782, 0.11752152218233858]
        arm.set_joint_value_target(joint_positions)

        # 控制机器人完成运动
        arm.go()
        r.sleep()

        arm.set_named_target("home")
        arm.go()
        r.sleep()

        moveit_commander.roscpp_shutdown()
    

if __name__ == "__main__":
    MoveitAttachedObjectDemo()