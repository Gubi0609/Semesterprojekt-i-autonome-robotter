#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



geometry_msgs::Pose getEEPose(tf2_ros::Buffer& tfBuffer) {
    geometry_msgs::Pose ee_pose;
    
    try {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("base_link", "tool0", ros::Time(0), ros::Duration(1.0));
        
        ee_pose.position.x = transformStamped.transform.translation.x;
        ee_pose.position.y = transformStamped.transform.translation.y;
        ee_pose.position.z = transformStamped.transform.translation.z;
        
        ee_pose.orientation = transformStamped.transform.rotation;
        
        ROS_INFO("Current EE Pose: x=%.3f, y=%.3f, z=%.3f", 
                 ee_pose.position.x, ee_pose.position.y, ee_pose.position.z);
        
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could not get transform: %s", ex.what());
    }

    return ee_pose;
}

double degToRad(double degrees) {
    return degrees * M_PI / 180.0;
}

void moveToPose(moveit::planning_interface::MoveGroupInterface& move_group, geometry_msgs::Pose target_pose) {
    move_group.setPoseTarget(target_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if (success) {
        ROS_INFO("Executing motion...");
        move_group.execute(plan);
    } else {
        ROS_WARN("Motion planning failed.");
    }
}

void moveToHomePosition(moveit::planning_interface::MoveGroupInterface& move_group) {
    // Define joint values for zero position {base, shoulder}

    std::vector<double> joint_group_positions = {degToRad(112.5), degToRad(-45), degToRad(-45), 0.0, 0.0, 0.0};

    // Set the joint values
    move_group.setJointValueTarget(joint_group_positions);

    // Plan and execute motion
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        ROS_INFO("Moving to home position...");
        move_group.execute(plan);
    } else {
        ROS_WARN("Motion planning failed.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ur5_moveit_control_tf2");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    ros::Duration(2.0).sleep();  // Ensure TF buffer is populated

    // Get the current end-effector pose
    geometry_msgs::Pose ee_pose = getEEPose(tfBuffer);
    moveToHomePosition(move_group);
    /*
    if (ee_pose.position.z != 0.0) {  // Check if pose is valid
        ee_pose.position.z += -0.1;  // Move 10 cm upwards
        moveToPose(move_group, ee_pose);
    }
    */

    ros::shutdown();
    return 0;
}
