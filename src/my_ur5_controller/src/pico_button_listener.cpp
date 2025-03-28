#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#define SERIAL_PORT "/dev/ttyACM0"

//To run script in terminal, use the following command:
//rosrun my_ur5_controller pico_button_listener

int main(int argc, char** argv) {
    ros::init(argc, argv, "pico_button_listener");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/ur5/move_z_up", 10);

    int serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        ROS_ERROR("Failed to open serial port %s", SERIAL_PORT);
        return -1;
    }

    struct termios options;
    tcgetattr(serial_fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(serial_fd, TCSANOW, &options);

    char buffer[256];
    while (ros::ok()) {
        int n = read(serial_fd, buffer, sizeof(buffer) - 1);
        if (n > 0) {
            buffer[n] = '\0';
            std::string input(buffer);
            ROS_INFO("Received from Pico: %s", input.c_str());

            if (input.find("MOVE_ARM") != std::string::npos) {
                std_msgs::String msg;
                msg.data = "move_up";
                ROS_INFO("Publishing: move_up");
                pub.publish(msg);
            }
        }
        ros::spinOnce();
    }

    close(serial_fd);
    return 0;
}
