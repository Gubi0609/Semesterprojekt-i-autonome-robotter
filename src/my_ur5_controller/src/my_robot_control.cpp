#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Trigger.h>

bool callService(const std::string& service_name)
{
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>(service_name);
    std_srvs::Trigger srv;
    if (client.call(srv))
    {
        ROS_INFO("%s", srv.response.message.c_str());
        return srv.response.success;
    }
    else
    {
        ROS_ERROR("Failed to call service: %s", service_name.c_str());
        return false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5_moveit_control");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Call Obstacle Manager Service to Add Obstacles
    if (!callService("add_obstacles"))
    {
        ROS_ERROR("Obstacle Manager service failed. Exiting...");
        return 1;
    }

    // Create MoveIt interface for UR5 manipulator
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    // Get the reference frame
    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Set planning parameters
    move_group.setPlannerId("RRTConnect");
    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);

    // Define target pose
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.4;
    target_pose.position.y = 0.2;
    target_pose.position.z = 0.3;
    target_pose.orientation.w = 1.0;

    // Set target pose and plan motion
    move_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        ROS_INFO("Motion plan successful! Waiting 5 seconds before execution...");
        ros::Duration(5.0).sleep();
        ROS_INFO("Executing motion...");
        move_group.execute(plan);
    }
    else
    {
        ROS_ERROR("Motion planning failed!");
    }

    // Call Obstacle Manager Service to Remove Obstacles
    callService("remove_obstacles");

    ros::shutdown();
    return 0;
}