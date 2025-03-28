#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_srvs/Trigger.h>

class ObstacleManager
{
public:
    ObstacleManager()
    {
        // Initialize service
        ros::NodeHandle nh;
        add_obstacle_service = nh.advertiseService("add_obstacles", &ObstacleManager::addObstacles, this);
        remove_obstacle_service = nh.advertiseService("remove_obstacles", &ObstacleManager::removeObstacles, this);
        ROS_INFO("Obstacle Manager is ready.");
    }

    bool addObstacles(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        std::vector<moveit_msgs::CollisionObject> collision_objects;

        // Define Box Obstacle
        moveit_msgs::CollisionObject box;
        box.header.frame_id = "world";  // Make sure this is the correct frame
        box.id = "box_1";

        shape_msgs::SolidPrimitive box_shape;
        box_shape.type = shape_msgs::SolidPrimitive::BOX;
        box_shape.dimensions = {0.3, 0.3, 0.3};

        geometry_msgs::Pose box_pose;
        box_pose.position.x = 0.5;
        box_pose.position.y = -0.2;
        box_pose.position.z = 0.15;

        box.primitives.push_back(box_shape);
        box.primitive_poses.push_back(box_pose);
        box.operation = moveit_msgs::CollisionObject::ADD;

        // Define Cylinder Obstacle
        moveit_msgs::CollisionObject cylinder;
        cylinder.header.frame_id = "world";
        cylinder.id = "cylinder_1";

        shape_msgs::SolidPrimitive cylinder_shape;
        cylinder_shape.type = shape_msgs::SolidPrimitive::CYLINDER;
        cylinder_shape.dimensions = {0.5, 0.1};

        geometry_msgs::Pose cylinder_pose;
        cylinder_pose.position.x = 0.3;
        cylinder_pose.position.y = 0.2;
        cylinder_pose.position.z = 0.25;

        cylinder.primitives.push_back(cylinder_shape);
        cylinder.primitive_poses.push_back(cylinder_pose);
        cylinder.operation = moveit_msgs::CollisionObject::ADD;

        collision_objects.push_back(box);
        collision_objects.push_back(cylinder);

        // Add obstacles to planning scene
        planning_scene_interface.addCollisionObjects(collision_objects);
        res.success = true;
        res.message = "Obstacles added successfully!";
        ROS_INFO("Added obstacles.");
        return true;
    }

    bool removeObstacles(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        std::vector<std::string> object_ids = {"box_1", "cylinder_1"};
        planning_scene_interface.removeCollisionObjects(object_ids);
        res.success = true;
        res.message = "Obstacles removed successfully!";
        ROS_INFO("Removed obstacles.");
        return true;
    }

private:
    ros::ServiceServer add_obstacle_service;
    ros::ServiceServer remove_obstacle_service;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_manager");
    ObstacleManager obstacle_manager;
    ros::spin();
    return 0;
}