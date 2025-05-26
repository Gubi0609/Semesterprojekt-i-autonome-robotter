#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Trigger.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// 1 tau rotation in radiants
const double tau = 2 * M_PI;

void openGripper(trajectory_msgs::JointTrajectory & posture)
{

}

void closedGripper(trajectory_msgs::JointTrajectory & posture)
{

}

void pick(moveit::planning_interface::MoveGroupInterface & move_group)
{

}

void place(moveit::planning_interface::MoveGroupInterface & group;)
{

}

void addCollisionObject(moveit::planning_interface::PlanningSceneInterface & planning_scene_interface)
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cobot_pick_and_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("arm");

    // Put the object in scene
    addCollisionObject(planning_scene_interface);

    // Wait for initilization
    ros::WallDuration(1.0).sleep();

    // Pick the object
    pick(group);

    ros::WallDuration(1.0).sleep()

    // Place the object
    place(group)

    ros::waitForShutdown();
    return 0;
}


