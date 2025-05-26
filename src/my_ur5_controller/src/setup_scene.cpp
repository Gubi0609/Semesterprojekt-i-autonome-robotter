#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shape_operations.h>

// To run program: rosrun my_ur5_controller setup_scene

void addTableToScene()
{
    // Create a PlanningSceneInterface object
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "world";  // Change if needed
    collision_object.id = "table";

    // Load the STL mesh
    shapes::Mesh* mesh = shapes::createMeshFromResource("package://ur5_moveit_config/meshes/table_mesh_scale.stl");
    shape_msgs::Mesh mesh_msg;
    shapes::ShapeMsg mesh_shape;
    shapes::constructMsgFromShape(mesh, mesh_shape);
    mesh_msg = boost::get<shape_msgs::Mesh>(mesh_shape);

    // Set mesh properties
    collision_object.meshes.push_back(mesh_msg);
    geometry_msgs::Pose mesh_pose;
    mesh_pose.position.x = -0.425;
    mesh_pose.position.y = -0.925;
    mesh_pose.position.z = -0.0305; // Adjust to match table height
    mesh_pose.orientation.w = 1.0;
    collision_object.mesh_poses.push_back(mesh_pose);

    // Add the object to the planning scene
    collision_object.operation = collision_object.ADD;
    planning_scene_interface.applyCollisionObject(collision_object);

    ROS_INFO("Table added to planning scene.");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_table_to_scene");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    addTableToScene();

    ros::shutdown();
    return 0;
}
