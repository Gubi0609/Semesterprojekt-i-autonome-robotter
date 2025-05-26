#include <ros/ros.h>
#include <std_msgs/String.h>
#include <moveit/move_group_interface/move_group_interface.h>

geometry_msgs::Pose getEEPose(moveit::planning_interface::MoveGroupInterface& move_group) {
    geometry_msgs::Pose ee_pose = move_group.getCurrentPose().pose;
    ROS_INFO("Current EE Pose: x=%.3f, y=%.3f, z=%.3f", 
             ee_pose.position.x, ee_pose.position.y, ee_pose.position.z);
    return ee_pose;
}

void move_up_callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received command: %s", msg->data.c_str());

    if (msg->data == "move_up") {
        moveit::planning_interface::MoveGroupInterface move_group("manipulator");

        // Debug: Print available joints
        std::vector<std::string> joint_names = move_group.getJoints();
        ROS_INFO("MoveIt knows about the following joints:");
        for (const auto& joint : joint_names) {
            ROS_INFO("- %s", joint.c_str());
        }

        // Get current pose
        geometry_msgs::Pose target_pose = getEEPose(move_group);

        // Validate pose
        if (target_pose.position.z == 0.0) {
            ROS_WARN("Invalid pose received! MoveIt may not have a valid robot state.");
            return;
        }

        // Move up by 10 cm
        target_pose.position.z += 0.10;
        move_group.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success) {
            ROS_INFO("Executing movement...");
            move_group.execute(plan);
            ROS_INFO("Moved UR5 up 10 cm");
        } else {
            ROS_WARN("Motion planning failed.");
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ur5_button_control");
    ros::NodeHandle nh;

    // Use MultiThreadedSpinner to avoid conflicts
    ros::AsyncSpinner spinner(2);  
    spinner.start();

    ROS_INFO("UR5 Button Control Node Started...");

    ros::Subscriber sub = nh.subscribe("/ur5/move_z_up", 10, move_up_callback);

    ros::waitForShutdown();  // Wait until ROS shutdown instead of using spin()
    return 0;
}
