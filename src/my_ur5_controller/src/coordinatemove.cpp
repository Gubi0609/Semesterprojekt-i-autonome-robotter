#include <ros/ros.h> // Standard ROS header file for initilizing and handling ROS nodes
#include <moveit/move_group_interface/move_group_interface.h> // Interface to moveit for planning and executing motion
#include <geometry_msgs/Pose.h> // Defines the Pose message type which holds position and orientation(quaternion) data
#include <vector>
#include <iostream>
#include <limits>
#include <array>
#include <string>
#include <utility>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <chrono>
#include <thread>



using namespace std;

//#ifndef _WIN32
#include "stockfishLinux.h"
#include "SerialPort.h"
//#endif

// To run the script, use the command:
// rosrun my_ur5_controller coordinatemove

struct PoseData {
    double x, y, z, qx, qy, qz, qw;
};

void move_to_pose(moveit::planning_interface::MoveGroupInterface &move_group, const PoseData& pose) 
{
    geometry_msgs::Pose target_pose; // Create a Pose object to hold the target pose data

    // Set TCP(Tool Center Point) position and orientation
    target_pose.position.x = pose.x;
    target_pose.position.y = pose.y;
    target_pose.position.z = pose.z;
    target_pose.orientation.x = pose.qx;
    target_pose.orientation.y = pose.qy;
    target_pose.orientation.z = pose.qz;
    target_pose.orientation.w = pose.qw;

    // Tells Moveit that the goal for the end effector is the target pose
    move_group.setPoseTarget(target_pose);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPlanningTime(10.0); // Set the maximum time Moveit is allowed to plan a motion
    
    // Plan and execute motion
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); //If succesful, success is true

    if (success) {
        ROS_INFO("Executing motion...");
        move_group.move();
    } else {
        ROS_ERROR("Motion planning failed.");
    }
}

double degToRad(double degrees)
{
    return degrees * M_PI / 180.0;
}
/*
void move_to_pose(moveit::planning_interface::MoveGroupInterface &move_group, const PoseData& pose) 
{
    geometry_msgs::Pose target_pose;
    target_pose.position.x = pose.x;
    target_pose.position.y = pose.y;
    target_pose.position.z = pose.z;
    target_pose.orientation.x = pose.qx;
    target_pose.orientation.y = pose.qy;
    target_pose.orientation.z = pose.qz;
    target_pose.orientation.w = pose.qw;

    move_group.setPoseTarget(target_pose);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPlanningTime(10.0);

    // ----- Shoulder joint constraint -----
    moveit_msgs::Constraints constraints;
    moveit_msgs::JointConstraint shoulder_constraint;
    shoulder_constraint.joint_name = "shoulder_lift_joint"; // Joint 2 (shoulder)
    shoulder_constraint.position = degToRad(-111); // Midpoint between -152 and -70
    shoulder_constraint.tolerance_above = degToRad(41); // 111 - 70
    shoulder_constraint.tolerance_below = degToRad(41); // 111 - (-152)
    shoulder_constraint.weight = 1.0;

    // --- Base joint constraint ---
    moveit_msgs::JointConstraint base;
    base.joint_name = "shoulder_pan_joint";
    base.position = degToRad(-62); // midpoint of -285 and -226
    base.tolerance_above = degToRad(29.5); // 255.5 - 226
    base.tolerance_below = degToRad(29.5); // 255.5 - 285
    base.weight = 1.0;
    constraints.joint_constraints.push_back(base);


    constraints.joint_constraints.push_back(shoulder_constraint);
    move_group.setPathConstraints(constraints);
    // -------------------------------------

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Remove path constraints after planning (or it stays active for next calls)
    //move_group.clearPathConstraints();

    if (success) {
        ROS_INFO("Executing motion...");
        move_group.move();
    } else {
        ROS_ERROR("Motion planning failed.");
    }
}

*/

geometry_msgs::Pose toGeometryPose(const PoseData& pose) {
    geometry_msgs::Pose msg;
    msg.position.x = pose.x;
    msg.position.y = pose.y;
    msg.position.z = pose.z;
    msg.orientation.x = pose.qx;
    msg.orientation.y = pose.qy;
    msg.orientation.z = pose.qz;
    msg.orientation.w = pose.qw;
    return msg;
}

bool move_cartesian_to_pose(moveit::planning_interface::MoveGroupInterface& move_group,
    const PoseData& target_pose_data)
{
// 1) Build the raw Cartesian path
geometry_msgs::Pose target_pose = toGeometryPose(target_pose_data);
std::vector<geometry_msgs::Pose> waypoints = {
move_group.getCurrentPose().pose,
target_pose
};
move_group.setMaxVelocityScalingFactor(0.05);
move_group.setMaxAccelerationScalingFactor(0.05);
const double eef_step       = 0.01;
const double jump_threshold = 0.0;

moveit_msgs::RobotTrajectory trajectory_msg;
double fraction = move_group.computeCartesianPath(
waypoints, eef_step, jump_threshold, trajectory_msg);

if (fraction < 0.99)
{
ROS_WARN("Cartesian path only %.1f%% – falling back to joint‐space",
fraction * 100.0);
move_group.setPoseTarget(target_pose);
moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
if (move_group.plan(joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
{
move_group.execute(joint_plan);
return true;
}
ROS_ERROR("  joint‐space fallback failed too");
return false;
}

// 2) Time‐parameterize via IPTP
robot_trajectory::RobotTrajectory rt(
move_group.getCurrentState()->getRobotModel(),
move_group.getName());
rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_msg);

trajectory_processing::IterativeParabolicTimeParameterization iptp;
bool ok = iptp.computeTimeStamps(rt);
if (!ok)
ROS_WARN("IPTP failed; will try to repair timestamps manually.");

// 3) Pull the (possibly timed) trajectory back out
rt.getRobotTrajectoryMsg(trajectory_msg);

// 4) **Enforce** strictly‐increasing times (epsilon = 1ms)
{
ros::Duration last(0.0);
const ros::Duration eps(0, 1000000);  // 0.001 s
for (auto& pt : trajectory_msg.joint_trajectory.points)
{
if (pt.time_from_start <= last)
pt.time_from_start = last + eps;
last = pt.time_from_start;
}
}

// 5) Execute
moveit::planning_interface::MoveGroupInterface::Plan timed_plan;
timed_plan.trajectory_ = trajectory_msg;
ROS_INFO("Executing Cartesian path (%.1f%% achieved)", fraction * 100.0);
move_group.execute(timed_plan);

return true;
}

void moveToHomePosition(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<double> joint_group_positions)
{
    // Define joint values for zero position {base, shoulder}

    //std::vector<double> joint_group_positions = {degToRad(112.5), degToRad(-45), degToRad(-45), 0.0, 0.0, 0.0};

    // Set the joint values
    ros::Duration(2.0).sleep();
    move_group.setJointValueTarget(joint_group_positions);

    // Plan and execute motion
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        ROS_INFO("Moving to start position...");
        move_group.execute(plan);
    } else {
        ROS_WARN("Motion planning failed.");
    }
}


std::array<std::array<PoseData, 8>, 8> create_chess_frame(const PoseData& starting_pose)
{
    const int rows = 8;
    const int cols = 8;

    std::array< std::array<PoseData, 8>, 8> grid; //create an empty vector to store the grid poses
    //grid.reserve(64); //Reserve space for 64 elements to avoid multiple memory allocations
    const double gridspace = 0.05;

    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            //PoseData pose = starting_pose;
            grid[i][j].x = starting_pose.x + i * gridspace;
            grid[i][j].y = starting_pose.y + j * -gridspace;
            grid[i][j].z = starting_pose.z;

            grid[i][j].qx = starting_pose.qx;
            grid[i][j].qy = starting_pose.qy;
            grid[i][j].qz = starting_pose.qz;
            grid[i][j].qw = starting_pose.qw;
        }
    }
    return grid;
}

std::array<std::array<std::string, 8>, 8> create_chesspiece_label_matrix()
{
    std::array<std::array<std::string, 8>, 8> board;
    board[0] = {"r", "n", "b", "q", "k", "b", "n", "r"};
    board[1] = {"p", "p", "p", "p", "p", "p", "p", "p"};

    for (int i = 2; i < 6; i++) {
        board[i] = {"", "", "", "", "", "", "", ""};
    }

    board[6] = {"p", "p", "p", "p", "p", "p", "p", "p"};
    board[7] = {"r", "n", "b", "q", "k", "b", "n", "r"};

    return board;
}

std::array<std::array<std::string, 8>, 8> create_color_label_matrix(bool is_player_white)
{
    //Set the color of the chess board
    //w for white and b for black
    std::array<std::array<std::string, 8>, 8> board;


    if (is_player_white)
    {
        board[0] = {"b", "b", "b", "b", "b", "b", "b", "b"};
        board[1] = {"b", "b", "b", "b", "b", "b", "b", "b"};
    
        for (int i = 2; i < 6; i++) {
            board[i] = {"", "", "", "", "", "", "", ""};
        }
    
        board[6] = {"w", "w", "w", "w", "w", "w", "w", "w"};
        board[7] = {"w", "w", "w", "w", "w", "w", "w", "w"};
    
        return board;
    }
    else
    {
        board[0] = {"w", "w", "w", "w", "w", "w", "w", "w"};
        board[1] = {"w", "w", "w", "w", "w", "w", "w", "w"};
    
        for (int i = 2; i < 6; i++) {
            board[i] = {"", "", "", "", "", "", "", ""};
        }
    
        board[6] = {"b", "b", "b", "b", "b", "b", "b", "b"};
        board[7] = {"b", "b", "b", "b", "b", "b", "b", "b"};
    
        return board;
    }
}

std::vector<std::vector<PoseData>> create_dynamic_loc_storage(const PoseData& starting_pose, int rows, int cols)
{

    const double gridspace = 0.05;
    // Function to store the pieces that have been removed from the board.
    // The function for creating and modifying the storage must be seperate functions
    // add_piece = {row, col}
    // remove_piece = {row, col}

    std::vector<std::vector<PoseData>> storage(rows, std::vector<PoseData>(cols));
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            storage[i][j].x = starting_pose.x + i * gridspace;
            storage[i][j].y = starting_pose.y + j * -gridspace;
            storage[i][j].z = starting_pose.z;
            storage[i][j].qx = starting_pose.qx;
            storage[i][j].qy = starting_pose.qy;
            storage[i][j].qz = starting_pose.qz;
            storage[i][j].qw = starting_pose.qw;
        }
    }
    return storage;
}

std::vector<std::vector<string>> create_dynamic_chess_storage(int rows, int cols)
{
    std::vector<std::vector<string>> storage(rows, std::vector<string>(cols));
    
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            storage[i][j] = "0";
        }
    }
    
    return storage;
}

// Rename this function to something more descriptive
void modify_player_storage(std::vector<std::vector<PoseData>> storage_cor, std::vector<std::vector<PoseData>> storage_cor_up, 
    std::vector<std::vector<string>>& storage, string target, int row, int col, 
    std::array<std::array<std::string, 8>, 8>& chess_board_pieces, moveit::planning_interface::MoveGroupInterface& move_group,
    SerialPort & serial)
{
    //&storage is a reference to the storage vector created in the function create_dynamic_chess_storage
    //target is the string that contains the target cell like a2a4
    //row and col are the row and column of the target cell
    //chess_board_pieces is the chess board matrix created in the function create_chesspiece_label_matrix

    //This function should be used to modify the storage vector created in the function create_storage
    //Storage here comes from create_dynamic_chess_storage


    for (int j = 0; j < storage[0].size(); j++)  // columns first
    {
        for (int i = 0; i < storage.size(); i++) // then rows
        {
            if (target.length() == 4)
            {
                if (storage[i][j] == "0")
                {
                    storage[i][j] = chess_board_pieces[row][col];
                    move_cartesian_to_pose(move_group, storage_cor_up[i][j]);
                    move_cartesian_to_pose(move_group, storage_cor[i][j]);
                    ros::Duration(1.0).sleep();
                    serial.openGripper();
                    ros::Duration(1.0).sleep();
                    move_cartesian_to_pose(move_group, storage_cor_up[i][j]);
                    return;
                }
            }
            else if (target.length() == 5)
            {
                if (storage[i][j][0] == target[target.size()-1])
                {
                    storage[i][j] = "0";
                    return;
                }
            }
        }
    }
}

void modify_robotpieces_storage(std::vector<std::vector<PoseData>> storage_cor, std::vector<std::vector<PoseData>> storage_cor_up, 
    std::vector<std::vector<string>>& storage, string target, int row, int col, 
    std::array<std::array<std::string, 8>, 8>& chess_board_pieces, moveit::planning_interface::MoveGroupInterface& move_group,
    SerialPort serial)
{
    for (int i = 0; i < storage.size(); i++)
    {
        for (int j = 0; j < storage[i].size(); j++)
        {
            if (target.length() == 4)
            {
                if (storage[i][j] == "0")
                {
                    storage[i][j] = chess_board_pieces[row][col];
                    return;
                }
            }
            else if (target.length() == 5)
            {
                if (storage[i][j][0] == target[target.size()-1])
                {
                    // Pick up the piece
                    move_cartesian_to_pose(move_group, storage_cor_up[i][j]);
                    move_cartesian_to_pose(move_group, storage_cor[i][j]);
                    // Gripper control
                    ros::Duration(1.0).sleep();
                    serial.closeGripper();
                    ros::Duration(1.0).sleep();
                    move_cartesian_to_pose(move_group, storage_cor_up[i][j]);
                    storage[i][j] = "0";
                    return;
                }
            }
            
        }
    }
}

void print_storage(const std::vector<std::vector<string>>& storage)
{
    for (int i = 0; i < storage.size(); i++)
    {
        for (int j = 0; j < storage[i].size(); j++)
        {
            std::cout << storage[i][j] << " ";
        }
        std::cout << std::endl;
    }
}


void dynamic_print_cor(const std::vector<std::vector<PoseData>>& storage)
{
    for (int i = 0; i < storage.size(); i++)
    {
        for (int j = 0; j < storage[i].size(); j++)
        {
            std::cout << "x: " << storage[i][j].x << " y: " << storage[i][j].y << " z: " << storage[i][j].z << std::endl;
        }
    }
}

void print_chessboard(const std::array<std::array<std::string, 8>, 8>& board)
{
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            if (board[i][j].empty()){
                std::cout << "0 ";
            } else {
                std::cout << board[i][j] << " ";
            }
        }
        std::cout << std::endl;
    }
}


std::array<std::array<std::string, 8>, 8> create_chess_label_matrix(bool is_player_white)
{
    std::array<std::array<std::string, 8>, 8> board;
    const std::string files[8] = {"a", "b", "c", "d", "e", "f", "g", "h"};

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            std::string file = files[j];
            int rank;

            if (is_player_white)
                rank = 8 - i;  // white sees rank 8 at top, 1 at bottom
            else
                rank = i + 1;  // black sees rank 1 at top, 8 at bottom

            board[i][j] = file + std::to_string(rank);
        }
    }

    return board;
}

void move_chess_piece_viz(std::array<std::array<std::string, 8>, 8>& boardcolor, std::array<std::array<std::string, 8>, 8>& board, int rowFrom, int colFrom, int rowTo, int colTo)
{
    std::string piece = board[rowFrom][colFrom];
    board[rowFrom][colFrom] = "";
    board[rowTo][colTo] = piece;
    boardcolor[rowFrom][colFrom] = "";
    boardcolor[rowTo][colTo] = piece;
}

std::vector<std::string> split(const std::string& input)
{
    std::vector<std::string> result;
    std::size_t lastPos = 0;
    // Using size_t instead of int is generally preferred when dealing with sizes and indices because size_t is specifically designed to represent the size of objects
    
    for (std::size_t i = 0; i < input.size(); ++i)
    {
        if (std::isdigit(input[i]))
        {
            if (i > lastPos)
            {
                result.push_back(input.substr(lastPos, i+1));
            }
            lastPos = i + 1;
        }
    }
    
    if (lastPos < input.size())
    {
        result.push_back(input.substr(lastPos));
    }
    return result;
}

// Returns (row, col) if found. Returns (-1, -1) if not found
std::pair<int, int> find_in_matrix(
    const std::array<std::array<std::string, 8>, 8>& matrix,
    //const std::vector<std::vector<std::string>>& storage,
    const std::string& target)
{
    for (int i = 0; i < matrix.size(); i++) {
        for (int j = 0; j < matrix[0].size(); j++) {
            if (matrix[i][j] == target) {
                return {i, j};  // found the target; return row & col
            }
        }
    }
    return {-1, -1};  // not found
}

PoseData offsetPoseZ(const PoseData& original, double offset_z)
{
    PoseData new_pose = original;
    new_pose.z += offset_z;
    return new_pose;
}

void castling(string target,
    bool &is_player_white,
    std::array<std::array<std::string, 8>, 8> &boardpieces,
    std::array<std::array<PoseData, 8>, 8> &boardloc_pick,
    moveit::planning_interface::MoveGroupInterface& move_group,
    std::array<std::array<PoseData, 8>, 8> &boardloc_move,
    std::array<std::array<std::string, 8>, 8>& chess_coordinatelabels,
    SerialPort &serial)
{
    // 1) If it's not black castling, do nothing.
    if (!((target == "e8g8" || target == "e8c8") && is_player_white))
    return;

    // 2) We know it's either e8g8 or e8c8, so split and lookup once.
    auto result      = split(target);
    auto [row, col]  = find_in_matrix(chess_coordinatelabels, result[0]);  // e8
    auto [row2, col2] = find_in_matrix(chess_coordinatelabels, result[1]);  // g8 or c8

    // 3) Castle the king
    move_cartesian_to_pose(move_group, boardloc_move[col][row]);
    move_cartesian_to_pose(move_group, boardloc_pick[col][row]);
    ros::Duration(1.0).sleep();
    serial.closeGripper();
    ros::Duration(1.0).sleep();
    move_cartesian_to_pose(move_group, boardloc_move[col][row]);
    move_cartesian_to_pose(move_group, boardloc_move[col2][row2]);
    move_cartesian_to_pose(move_group, boardloc_pick[col2][row2]);
    ros::Duration(1.0).sleep();
    serial.openGripper();
    ros::Duration(1.0).sleep();
    move_cartesian_to_pose(move_group, boardloc_move[col2][row2]);

    // 4) Then castle the rook
    if (target == "e8g8") {
    // rook h8→f8
    move_cartesian_to_pose(move_group, boardloc_move[7][0]);
    move_cartesian_to_pose(move_group, boardloc_pick[7][0]);
    ros::Duration(1.0).sleep();
    serial.closeGripper();
    ros::Duration(1.0).sleep();
    move_cartesian_to_pose(move_group, boardloc_move[7][0]);
    move_cartesian_to_pose(move_group, boardloc_move[5][0]);
    move_cartesian_to_pose(move_group, boardloc_pick[5][0]);
    ros::Duration(1.0).sleep();
    serial.openGripper();
    ros::Duration(1.0).sleep();
    move_cartesian_to_pose(move_group, boardloc_move[5][0]);
    
    } else if (target == "e8c8") {
    // rook a8→d8
    move_cartesian_to_pose(move_group, boardloc_move[0][0]);
    move_cartesian_to_pose(move_group, boardloc_pick[0][0]);
    ros::Duration(1.0).sleep();
    serial.closeGripper();
    ros::Duration(1.0).sleep();
    move_cartesian_to_pose(move_group, boardloc_move[0][0]);
    move_cartesian_to_pose(move_group, boardloc_move[3][0]);
    move_cartesian_to_pose(move_group, boardloc_pick[3][0]);
    ros::Duration(1.0).sleep();
    serial.openGripper();
    ros::Duration(1.0).sleep();
    move_cartesian_to_pose(move_group, boardloc_move[3][0]);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "coordinatemove"); // Initialize the ROS node with the name "coordinatemove"
    ros::NodeHandle node_handle; //Creates a ROS node handle to communicate with the ROS system

    //starts a background thread for handling the ROS communication events as moveit listens for feedback asynchronously
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    // Create a MoveGroupInterface object for the manipulator group
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    // Set initial position for the robot
    std::vector<PoseData> poses = {
        {-0.200, -0.444, 0.197, 0, 1, 0, 0}, // Initial position for chess board
        {-0.351, -0.444, 0.150, 0, 1, 0, 0}, // Initial position for white storage
        {0.300, -0.444, 0.150, 0, 1, 0, 0}, // Initial position for black storage
        {-0.200, -0.444, 0.339, 0, 1, 0, 0}, // Offset for moving pieces
        {-0.351, -0.444, 0.339, 0, 1, 0, 0}, // Offset for moving white storage pieces
        {0.300, -0.444, 0.339, 0, 1, 0, 0} // Offset for moving black storage pieces
    };

    //--------------------------------Select color--------------------------------

    // Player or computer start
    bool is_player_white = true; // true if player is white, false if player is black
    bool is_player_selected = false; // true if player has selected a color, false otherwise
    bool is_players_turn = true; // true if it is the player's turn, false if it is the computer's turn

    while(is_player_selected==false)
    {
        cout << "Do you wish to play as white or black? <w/b>";
        string color;
        cin >> color;
        if (color == "w")
        {
            is_player_white = true;
            is_player_selected = true;
            is_players_turn = true;
        }
        else if (color == "b")
        {
            is_player_white = false;
            is_player_selected = true;
            is_players_turn = false;
        }
        else 
        {
            cout << "Invalid colour choice. Please choose either w or b." << endl;
            continue;
        }
    }

    //------------------------------------Set game board-------------------------------

    std::vector<double> joint_group_positions = {degToRad(-59), degToRad(-73), degToRad(-117), degToRad(-80), degToRad(90), degToRad(18)};

    moveToHomePosition(move_group, joint_group_positions);

    // Create coordinate system for robot
    auto grid_pick = create_chess_frame(poses[0]);
    auto grid_move = create_chess_frame(poses[3]);
    // Create chess coordinate labels (A1, B1, C1, etc.)
    auto chess_coordinatelabels = create_chess_label_matrix(is_player_white);
    // Create chess board pieces (r, n, b, q, k, p)
    auto chess_board_pieces = create_chesspiece_label_matrix();
    // Create chess white storage system
    auto white_storage_cor = create_dynamic_loc_storage(poses[1], 2, 8);
    auto white_storage_cor_up = create_dynamic_loc_storage(poses[4], 2, 8);
    //auto black_storage_cor = create_dynamic_loc_storage(poses[2], 2, 8);
    auto white_storage = create_dynamic_chess_storage(2, 8);
    // Create chess black storage system
    auto black_storage_cor_up = create_dynamic_loc_storage(poses[2], 2, 8);
    auto black_storage_cor = create_dynamic_loc_storage(poses[5], 2, 8);
    auto black_storage = create_dynamic_chess_storage(2, 8);
    // Create chess board color labels (b, w)
    auto chess_board_color = create_color_label_matrix(is_player_white);

    //------------------------------Stockfish--------------------------------
    //#ifndef _WIN32
    StockfishLinux engine("/home/magnusm/ur5_ws/src/my_ur5_controller/src/stockfish-android-armv8", 3);
    //#endif
    
    const char* port = "/dev/ttyACM0";  // Adjust this to your serial port device.
    SerialPort serial;

    if (serial.openSerial(port) < 0) {
        return 1;
    }

    std::cout << "Serial port opened successfully." << std::endl;

    //To get tool flange location
    //rosrun tf tf_echo /base_link /tool0


    //Gripper control
    
    //serial.openGripper();
    //std::this_thread::sleep_for(std::chrono::seconds(2));
    //serial.closeGripper();
    

    //Missing features:
    /*
    Choose first player and thereby rotate or not the frames !DONE!
    How does stockfish make the first move and how do i tell it what color it is? !DONE!
    When the player decommisions computers pieces, create function for filling computer storage !DONE!
    Handle promotions, retrieve from storage !DONE!

    Flyt 1 felt brættet mod magne gruppen( 5 cm på x aksen, også på white storage)
    Handle castling 
    Handle game ending, this happens when there are no legal moves left

    Handle illegal moves
    Gripper control in modify storage function
    empassant move
    Ret robottens start position
    Add a queen to the start of each storage

    Lav trajectory graf

    Getbestmove function, castling e1c1/e1g1/e8g8/e8c8

    */


    /*
    
    std::cout << "Chosen cell: row = " << row << ", col = " << col << std::endl;
    std::cout << "Pose: x=" << grid[row][col].x 
          << " y=" << grid[row][col].y 
          << " z=" << grid[row][col].z 
          << " qx=" << grid[row][col].qx
          << " qy=" << grid[row][col].qy
          << " qz=" << grid[row][col].qz
          << " qw=" << grid[row][col].qw << std::endl;

    */

    

    while (ros::ok())
    {
        moveToHomePosition(move_group, joint_group_positions);
        std::cout << "Chess board pieces:" << std::endl;
        print_chessboard(chess_board_pieces);
        std::cout << "Coordinate levels" << std::endl;
        print_chessboard(chess_coordinatelabels);
        std::cout << "Chess board color:" << std::endl;
        print_chessboard(chess_board_color);
        std::cout << "White storage: " << std::endl;
        print_storage(white_storage);
        std::cout << "Black storage: " << std::endl;
        print_storage(black_storage);
        

        while(false)
        {
            std::vector<std::string> legalMoves = engine.getLegalMoves();
            serial.sendlegalmoves(legalMoves);
            string target=serial.movefrompico(); //cin >> target;
            std::cout << "Data received from port: " << target << std::endl;
        }

        //serial.closeGripper();
        //std::this_thread::sleep_for(std::chrono::seconds(2));
        //serial.openGripper();
        //std::this_thread::sleep_for(std::chrono::seconds(2));
        //serial.openGripper();

        while(false)
        {
            std::array<std::string, 8> targets = {"a1a1", "b1b1", "c1c1", "d1d1", "e1e1", "f1f1", "g1g1", "h1h1"};            
            for (int i = 0; i < targets.size(); i++)
            {
                cout << "Print" << targets[i] << std::endl;
                std::vector<std::string> result = split(targets[i]);
                cout << "Result: " << result[0] << " " << result[1] << std::endl;
                auto [row, col] = find_in_matrix(chess_coordinatelabels, result[0]);
                auto [row2, col2] = find_in_matrix(chess_coordinatelabels, result[1]);

                modify_player_storage(
                    white_storage_cor,      // storage_cor
                    white_storage_cor_up,   // storage_cor_up
                    white_storage,          // storage (vector<vector<string>>)
                    targets[i],                 // target
                    row2, 
                    col2, 
                    chess_board_pieces, 
                    move_group,
                    serial
                ); 
                print_storage(white_storage);
            }         
        }

        // 2 åbner først på vej op og 1 gang preemted fejl, 47 forsøg med ingen fejl,
        // Problemet ligner at det fremkommer ved længere banere, flere linjer af move...
        while(true)
        {
            string target;
            cout << "Enter move down/up?" << endl;
            cin >> target;
            if (target=="d")
            {
                move_cartesian_to_pose(move_group, poses[0]);
                serial.closeGripper();
                ros::Duration(1.0).sleep();
                move_cartesian_to_pose(move_group, poses[3]);
                move_cartesian_to_pose(move_group, poses[0]);
                serial.openGripper();
                move_cartesian_to_pose(move_group, poses[3]);
            }
            if (target == "up")
            {
                move_cartesian_to_pose(move_group, poses[3]);
                
            }
            if (target == "close")
            {
                serial.closeGripper();
            }
            if (target == "open")
            {
                serial.openGripper();
                move_cartesian_to_pose(move_group, poses[3]);
            }
        }

        //move_cartesian_to_pose(move_group, poses[0]);
        

        std::cout << "Player is white? " << is_player_white << std::endl;

        std::string target;
        std::string bestmove;

        if (is_players_turn) //Same as (is_player_turn == true)
        {
            std::vector<std::string> legalMoves = engine.getLegalMoves();
            serial.stopGripper();
            ros::Duration(.5).sleep();
            serial.sendlegalmoves(legalMoves);
            cout << "Make move" << endl;
            target=serial.movefrompico(); //cin >> target; to play by computer
            std::cout << "Data received from port: " << target << std::endl;

            //std::cin >> target;
            engine.appendMovesMade(target);
            is_players_turn = false;
            std::cout << "Player's turn " << is_players_turn << std::endl;
        }
        else
        {
            cout << "Computer executing move" << endl;
            target = engine.getBestMove();
            std::cout << "Best move: " << bestmove << std::endl;
            engine.appendMovesMade(target);
            is_players_turn = true;
            std::cout << "Player's turn " << is_players_turn << std::endl;
        }

        //std::string target;
        //std::cout << "Enter piece pick and place location: <a1e2>";
        //std::cin >> target;
        //std::vector<std::string> result = split(target);

        //std::vector<std::string> legalMoves = engine.getLegalMoves();
        
        //serial.sendlegalmoves(legalMoves);

        //string target = serial.movefrompico();
        
        //Spiller move er target



        //Tell stockfish the move that was made
        //engine.appendMovesMade(target);

        //Get the move from stockfish
        //std::string bestmove = engine.getBestMove();
        //std::cout << "Best move: " << bestmove << std::endl;

        cout << "Print" << target << std::endl;
        std::vector<std::string> result = split(target);

        cout << "Result: " << result[0] << " " << result[1] << std::endl;

        auto [row, col] = find_in_matrix(chess_coordinatelabels, result[0]);
        auto [row2, col2] = find_in_matrix(chess_coordinatelabels, result[1]);

        if (chess_board_pieces[row2][col2] != "" && is_players_turn == true) // && is_player_turn == true
        {   
            std::cout << "Piece already exists in target cell: " << result[1] << std::endl;
            //Moving to pick up
            cout << "Moving to pick up sacrificed piece in move matrix" << endl;
            move_cartesian_to_pose(move_group, grid_move[col2][row2]);
            cout << "Moving to pick up sacrificed piece in pick matrix" << endl;
            move_cartesian_to_pose(move_group, grid_pick[col2][row2]);
            ros::Duration(1.0).sleep();
            serial.closeGripper();
            ros::Duration(1.0).sleep();
            //Moving to storage
            cout << "Moving to storage in move matrix" << endl;
            move_cartesian_to_pose(move_group, grid_move[col2][row2]);
            modify_player_storage(
                white_storage_cor,      // storage_cor
                white_storage_cor_up,   // storage_cor_up
                white_storage,          // storage (vector<vector<string>>)
                target,                 // target
                row2, 
                col2, 
                chess_board_pieces, 
                move_group,
                serial
            );            
            move_chess_piece_viz(chess_board_color, chess_board_pieces, row, col, row2, col2);
            //moving to place
            move_cartesian_to_pose(move_group, grid_move[col][row]);
            move_cartesian_to_pose(move_group, grid_pick[col][row]);
            // Gripper control
            ros::Duration(1.0).sleep();
            serial.closeGripper();
            ros::Duration(1.0).sleep();
            move_cartesian_to_pose(move_group, grid_move[col][row]);
            // moving to pick up
            move_cartesian_to_pose(move_group, grid_move[col2][row2]);
            move_cartesian_to_pose(move_group, grid_pick[col2][row2]);
            ros::Duration(1.0).sleep();
            serial.openGripper();
            ros::Duration(1.0).sleep();
            //move to above previous position
            move_cartesian_to_pose(move_group, grid_move[col2][row2]);
        }
        else if (chess_board_pieces[row2][col2] == "" && is_players_turn == true) // && is_player_turn == true
        {
            castling(target, is_player_white, chess_board_pieces, grid_pick, move_group, grid_move, chess_coordinatelabels, serial);
            std::cout << "Moving piece from " << result[0] << " to " << result[1] << std::endl;
            move_chess_piece_viz(chess_board_color, chess_board_pieces, row, col, row2, col2);
            //Moving to pick up
            cout << "Moving to pick up sacrificed piece in move matrix" << endl;
            move_cartesian_to_pose(move_group, grid_move[col][row]);
            cout << "Moving to pick up sacrificed piece in pick matrix" << endl;
            move_cartesian_to_pose(move_group, grid_pick[col][row]);
            ros::Duration(1.0).sleep();
            serial.closeGripper();
            ros::Duration(1.0).sleep();
            cout << "Moving to pick up sacrificed piece in move matrix" << endl;
            move_cartesian_to_pose(move_group, grid_move[col][row]);
            //ros::Duration(2.0).sleep();
            cout << "Moving to pick up sacrificed piece in move matrix" << endl;
            move_cartesian_to_pose(move_group, grid_move[col2][row2]);
            cout << "Moving to pick up sacrificed piece in pick matrix" << endl;
            move_cartesian_to_pose(move_group, grid_pick[col2][row2]);
            ros::Duration(1.0).sleep();
            serial.openGripper();
            ros::Duration(1.0).sleep();
            cout << "Moving to pick up sacrificed piece in move matrix" << endl;
            move_cartesian_to_pose(move_group, grid_move[col2][row2]);
        }
        else if(chess_board_pieces[row2][col2] != "" && is_players_turn == false)
        {
            cout << "Player is executing move, updating chessboard" << endl;
            cout << "Piece already exists in target cell: " << result[1] << endl;
            //move to storage
            modify_robotpieces_storage(
                black_storage_cor,      // storage_cor
                black_storage_cor_up,   // storage_cor_up
                black_storage,          // storage (vector<vector<string>>)
                target,                 // target
                row2, 
                col2, 
                chess_board_pieces, 
                move_group,
                serial
            );
            //modify in-game chess board
            move_chess_piece_viz(chess_board_color, chess_board_pieces, row, col, row2, col2);
            //moving to place
            //move_to_pose
        }
        else if(chess_board_pieces[row2][col2] == "" && is_players_turn == false)
        {
            cout << "Player is executing move, updating chessboard" << endl;
            cout << "Player moving piece from " << result[0] << " to " << result[1] << endl;
            move_chess_piece_viz(chess_board_color, chess_board_pieces, row, col, row2, col2);
        }

        else if (row == -1 && col == -1) {
            std::cout << "Invalid cell: " << target << std::endl;
            continue;
        }
    }
    ROS_INFO("Movement complete!");
    ros::shutdown();
    return 0;
}