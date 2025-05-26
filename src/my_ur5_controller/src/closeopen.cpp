#include <iostream>
#include <string>
#include <fcntl.h>      // For open() flags
#include <termios.h>    // For terminal I/O
#include <unistd.h>     // For write() and close()
#include <cstring>      // For memset()

int main() {
    // Specify the serial port device. Adjust this as needed.
    const char *serialPort = "/dev/ttyACM0"; // or "/dev/ttyUSB0"

    // Open the serial port device (it remains open throughout the program).
    int fd = open(serialPort, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening serial port " << serialPort << std::endl;
        return 1;
    }

    // Configure the serial port.
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr" << std::endl;
        close(fd);
        return 1;
    }

    // Set baud rate to 9600 for both input and output.
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    // Configure for 8-bit characters, no parity, 1 stop bit.
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // non-blocking read
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff flow control
    tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);    // shut off parity
    tty.c_cflag &= ~CSTOPB;               // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;              // no hardware flow control

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr" << std::endl;
        close(fd);
        return 1;
    }

    // Main loop: continuously prompt for user input and send the corresponding command.
    std::string input;
    std::cout << "Enter command (\"close\" to send 1, \"open\" to send 0). Type \"exit\" to quit:" << std::endl;
    
    while (true) {
        std::cout << "> ";
        std::getline(std::cin, input);

        if (input == "exit") {
            break;
        } else if (input == "close") {
            char cmd = '1';
            ssize_t n = write(fd, &cmd, 1);
            if (n < 0) {
                std::cerr << "Error writing to serial port" << std::endl;
            } else {
                std::cout << "Sent: " << cmd << std::endl;
            }
        } else if (input == "open") {
            char cmd = '0';
            ssize_t n = write(fd, &cmd, 1);
            if (n < 0) {
                std::cerr << "Error writing to serial port" << std::endl;
            } else {
                std::cout << "Sent: " << cmd << std::endl;
            }
        } else {
            std::cout << "Invalid command. Only \"close\" and \"open\" are accepted." << std::endl;
        }
    }

    close(fd);
    return 0;
}
