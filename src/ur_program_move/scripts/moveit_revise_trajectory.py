import rospy
import sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


def scale_trajectory_speed(traj : RobotTrajectory, scale_factor):
    # 创建一个新的轨迹对象
    new_traj = RobotTrajectory()
    # 初始化new_traj的关节轨迹和traj的一样
    new_traj.joint_trajectory = traj.joint_trajectory
    # 获取关节数量
    n_joints = len(traj.joint_trajectory.joint_names)
    # 获取点的数量
    n_points = len(traj.joint_trajectory.points)
    # 存储轨迹点列表
    points = list(traj.joint_trajectory.points)

    for i in range(n_points):
        point = JointTrajectoryPoint()
        point.positions = traj.joint_trajectory.points[i].positions
        # time_from_start是轨迹开始到当前轨迹点的时间
        # 速度和加速度变慢了，因此time_from_start应该要变快
        point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale_factor
        point.velocities = list(traj.joint_trajectory.points[i].velocities)
        point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
        for j in range(n_joints):
            point.velocities[j] *= scale_factor
            point.accelerations[j] *= scale_factor * scale_factor
        
        points[i] = point

    new_traj.joint_trajectory.points = points
    return new_traj


class MoveSpeedDemo:
    """
    轨迹的修改（速度变换）
    使用：
    1. roslaunch ur_moveit_config demo.launch
    2 rosrun ur_program_move moveit_revise_trajectory.py
    """
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_speed")
        r = rospy.Rate(1)
        arm = MoveGroupCommander("ur_arm")
        
        end_effector_link = arm.get_end_effector_link()
        arm.allow_replanning(True)
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.01)
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        arm.set_named_target("home")
        arm.go()

        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        joint_positions = [0.391410, -0.676384, -0.376217, 0.0, 1.052834, 0.454125]
        arm.set_joint_value_target(joint_positions)
        arm.go()

        arm.set_named_target('home')
        arm.go()


        # Get back the planned trajectory
        arm.set_joint_value_target(joint_positions)
        plan_success, traj, planning_time, error_code = arm.plan()
        # 修改速度
        new_traj = scale_trajectory_speed(traj, 0.25)

        arm.execute(new_traj)
        r.sleep(1)

        arm.set_named_target("home")
        arm.go()

        moveit_commander.roscpp_shutdown()

    
if __name__ == "__main__":
    MoveSpeedDemo()