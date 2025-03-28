#include <ros/ros.h> // Standard ROS header file for initilizing and handling ROS nodes
#include <moveit/move_group_interface/move_group_interface.h> // Interface to moveit for planning and executing motion
#include <geometry_msgs/Pose.h> // Defines the Pose message type which holds position and orientation(quaternion) data
#include <vector>
#include <iostream>
#include <limits>
#include <array>
#include <string>
#include <utility>
using namespace std;

//#ifndef _WIN32
#include "stockfishLinux.h"
//#endif

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

void moveToHomePosition(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<double> joint_group_positions)
{
    // Define joint values for zero position {base, shoulder}

    //std::vector<double> joint_group_positions = {degToRad(112.5), degToRad(-45), degToRad(-45), 0.0, 0.0, 0.0};

    // Set the joint values
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

std::vector<std::vector<string>> create_color_label_matrix(std::string start_color)
{
    //Set the color of the chess board
    //w for white and b for black
    std::vector<std::vector<string>> board(8, std::vector<string>(8));


    if (start_color == "w")
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
    else if (start_color == "b")
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

    std::vector<std::vector<string>> board(8, std::vector<string>(8));

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


void modify_storage(std::vector<std::vector<PoseData>> storage_cor,  
    std::vector<std::vector<string>>& storage, string target, int row, int col, 
    std::array<std::array<std::string, 8>, 8>& chess_board_pieces, moveit::planning_interface::MoveGroupInterface& move_group)
{
    //&storage is a reference to the storage vector created in the function create_dynamic_chess_storage
    //target is the string that contains the target cell like a2a4
    //row and col are the row and column of the target cell
    //chess_board_pieces is the chess board matrix created in the function create_chesspiece_label_matrix

    //This function should be used to modify the storage vector created in the function create_storage
    //Storage here comes from create_dynamic_chess_storage


    for (int i = 0; i < storage.size(); i++)
    {
        for (int j = 0; j < storage[i].size(); j++)
        {
            if (target.length() == 4)
            {
                if (storage[i][j] == "0")
                {
                    storage[i][j] = chess_board_pieces[row][col];
                    move_to_pose(move_group, storage_cor[i][j]);
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


std::array<std::array<std::string, 8>, 8> create_chess_label_matrix(string start_color)
{
    std::array<std::array<std::string, 8>, 8> board;
    if(start_color == "w")
    {
        // Each row i → rank = i+1 (1..8)
        // Each column j → file = 'A' + j (A..H)
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                char file = 'a' + j;         // 'A'..'H'
                int rank = i + 1;           // 1..8
                board[i][j] = std::string(1, file) + std::to_string(rank);
                // e.g. row=0,col=0 => "A1", row=7,col=7 => "H8"
                }
            }
        return board;
    }

    else if(start_color == "b")
    {
        // Each row i → rank = i+1 (1..8)
        // Each column j → file = 'A' + j (A..H)
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                char file = 'h' - j;         // 'A'..'H'
                int rank = 8 - i;           // 1..8
                board[i][j] = std::string(1, file) + std::to_string(rank);
                // e.g. row=0,col=0 => "A1", row=7,col=7 => "H8"
            }
        }
        return board;
    }
}

void move_chess_piece_viz(std::array<std::array<std::string, 8>, 8>& board, int rowFrom, int colFrom, int rowTo, int colTo)
{
    std::string piece = board[rowFrom][colFrom];
    board[rowFrom][colFrom] = "";
    board[rowTo][colTo] = piece;
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
        {-0.200, -0.444, 0.151, 0, 1, 0, 0}, // Initial position for chess board
        {-0.351, -0.444, 0.151, 0, 1, 0, 0}, // Initial position for white storage
        {0.300, -0.444, 0.151, 0, 1, 0, 0}, // Initial position for black storage
        {-0.200, -0.444, 0.151+0.1, 0, 1, 0, 0} // Offset for moving pieces
    };

    std::vector<double> joint_group_positions = {degToRad(95), degToRad(-82), degToRad(111), degToRad(-120), degToRad(-90), degToRad(326)};
    moveToHomePosition(move_group, joint_group_positions);

    auto grid_pick = create_chess_frame(poses[0]);
    auto grid_move = create_chess_frame(poses[3]);
    auto chess_coordinatelabels = create_chess_label_matrix("w");
    auto chess_board_pieces = create_chesspiece_label_matrix();
    auto white_storage_cor = create_dynamic_loc_storage(poses[1], 2, 8);
    auto black_storage_cor = create_dynamic_loc_storage(poses[2], 2, 8);
    auto white_storage = create_dynamic_chess_storage(2, 8);

    //------------------------------Stockfish--------------------------------
    //#ifndef _WIN32
    StockfishLinux engine("/home/magnusm/ur5_ws/src/my_ur5_controller/src/stockfish-android-armv8", 3);
    //#endif
    


    /*
    // Set the target pose
    int row;
    int col;
    std::cout << "Enter the row of the pose you want to move to: ";
    std::cin >> row;
    std::cout << "Enter the  column of the pose you want to move to: ";
    std::cin >> col;

    
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
        
        print_chessboard(chess_board_pieces);
        print_chessboard(chess_coordinatelabels);
        print_storage(white_storage);
        

        /*
        string option;
        std::cout << "Move to a specific coordinate or play chess? <move/chess>";
        std::cin >> option;

        if (option == "move")
        {
            cout << "Enter the coordinate you wish to move to: ";
            string ;
        }
        */

        /*
        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                move_to_pose(move_group, white_storage[j][i]);
                ros::Duration(0.5).sleep();
            }
        }
        */
        /*
        int i, j;
        std::cout <<"Enter row: ";
        cin >> i;
        std::cout << "Enter col: ";
        cin >> j;

        dynamic_print(white_storage_cor);
        
        move_to_pose(move_group, white_storage_cor[j][i]);
        */
        
        
        std::string target;
        std::cout << "Enter piece pick and place location: <a1e2>";
        std::cin >> target;
        //std::vector<std::string> result = split(target);

        //Tell stockfish the move that was made
        engine.appendMovesMade(target);

        //Get the move from stockfish
        std::string bestmove = engine.getBestMove();
        std::cout << "Best move: " << bestmove << std::endl;

        target = bestmove;
        std::vector<std::string> result = split(target);

        auto [row, col] = find_in_matrix(chess_coordinatelabels, result[0]);
        auto [row2, col2] = find_in_matrix(chess_coordinatelabels, result[1]);

        if (chess_board_pieces[row2][col2] != "") 
        {

            std::cout << "Piece already exists in target cell: " << result[1] << std::endl;
            //Moving to pick up
            move_to_pose(move_group, grid_move[col2][row2]);
            move_to_pose(move_group, grid_pick[col2][row2]);
            //Moving to storage
            modify_storage(white_storage_cor, white_storage, target, row2, col2, chess_board_pieces, move_group);
            move_chess_piece_viz(chess_board_pieces, row, col, row2, col2);
            //ros::Duration(2.0).sleep();
            //moving to place
            move_to_pose(move_group, grid_move[col][row]);
            move_to_pose(move_group, grid_pick[col][row]);
            //move_to_pose(move_group, grid[col][row]);
            move_to_pose(move_group, grid_move[col2][row2]);
            move_to_pose(move_group, grid_pick[col2][row2]);
        }
        if (chess_board_pieces[row2][col2] == "")
        {
            std::cout << "Moving piece from " << result[0] << " to " << result[1] << std::endl;
            move_chess_piece_viz(chess_board_pieces, row, col, row2, col2);
            move_to_pose(move_group, grid_pick[col][row]);
            //ros::Duration(2.0).sleep();
            move_to_pose(move_group, grid_pick[col2][row2]);
        }
        

        if (row == -1 && col == -1) {
            std::cout << "Invalid cell: " << target << std::endl;
            continue;
        }
        
        

    }


    ROS_INFO("Movement complete!");
    ros::shutdown();
    return 0;

}