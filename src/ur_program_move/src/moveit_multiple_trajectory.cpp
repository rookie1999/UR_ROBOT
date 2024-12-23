#include "ros/ros.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"
#include "moveit_msgs/OrientationConstraint.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "moveit_multiple_trajectory");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);  // 创建一个异步spinner，参数1表示使用1个线程
    spinner.start();  // 启动spinner，允许异步处理回调函数

    moveit::planning_interface::MoveGroupInterface arm("ur_arm");
    arm.setGoalJointTolerance(0.01);
    double accScale = 0.8;
    double velScale = 0.8;
    arm.setMaxAccelerationScalingFactor(accScale);
    arm.setMaxVelocityScalingFactor(velScale);

    // 控制机械臂回到初始位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    // 获取机械臂的起始位置
    robot_state::RobotStatePtr start_state(arm.getCurrentState());
    // 从当前状态中获取与机械臂名称对应的关节模型组
    const robot_state::JointModelGroup * joint_model_group = start_state->getJointModelGroup(arm.getName());

    // 创建一个向量来存储关节组的位置，并从当前状态中复制这些位置
    std::vector<double> joint_group_positions;
    start_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // 设置第一个目标点
    joint_group_positions[0] = -0.6; // 弧度
    arm.setJointValueTarget(joint_group_positions);
    // 计算第一条轨迹(创建一个计划对象，并计算从当前状态到目标位置的轨迹)
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    robot_state::MoveItErrorCode success = arm.plan(plan1);
    
    // 重新设置关节模型组和起始状态，以便进行下一次规划
    // 即下一次规划的起点是第一次规划的终点
    joint_model_group = start_state->getJointModelGroup(arm.getName());
    start_state->setJointGroupPositions(joint_model_group, joint_group_positions);
    arm.setStartState(*start_state);

    //设置第二个目标点
    joint_group_positions[0] = -1.2;  // radians
    joint_group_positions[1] = -0.5;  // radians
    arm.setJointValueTarget(joint_group_positions);
    // 计算第二条轨迹
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    success = arm.plan(plan2);

    // 连接两条轨迹
    moveit_msgs::RobotTrajectory traj;
    // 将第1条轨迹的关节名称和点复制到合并后的轨迹中
    traj.joint_trajectory.joint_names = plan1.trajectory_.joint_trajectory.joint_names;
    traj.joint_trajectory.points = plan1.trajectory_.joint_trajectory.points;
    // 将第2条轨迹的关节名称和点复制到合并后的轨迹中
    // 不包括第一个点，因为第一个点与前一个轨迹的最后一个点重合
    for (size_t j = 1; j < plan2.trajectory_.joint_trajectory.points.size(); j++) {
        traj.joint_trajectory.points.push_back(plan2.trajectory_.joint_trajectory.points[j]);
    }

    // 重规划
    // 创建RobotTrajectory对象，用于处理合并后的轨迹
    // 将合并后的轨迹设置到RobotTrajectory对象中
    robot_trajectory::RobotTrajectory rt(arm.getCurrentState()->getRobotModel(), "ur_arm");
    rt.setRobotTrajectoryMsg(*arm.getCurrentState(), traj);
    // 使用IterativeParabolicTimeParameterization算法来计算合并后轨迹的时间戳，考虑速度和加速度缩放因子
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, velScale, accScale);

    // 将RobotTrajectory对象中的轨迹提取出来，并设置到计划对象中
    rt.getRobotTrajectoryMsg(traj);
    // 一个新的计划对象
    moveit::planning_interface::MoveGroupInterface::Plan joinedPlan;
    joinedPlan.trajectory_ = traj;

    if (!arm.execute(joinedPlan)) {
        ROS_ERROR("Fail to execute the plan!");
        return false;
    }
    sleep(1);
    ROS_INFO("Finished");

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);
    ros::shutdown();

    return 0;
}
