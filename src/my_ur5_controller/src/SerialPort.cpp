#include "SerialPort.h"
#include <fcntl.h>      // For open() flags
#include <termios.h>    // For terminal I/O
#include <unistd.h>     // For write() and close()
#include <cstring>      // For memset()
#include <iostream>

// Constructor: initialize file descriptor.
SerialPort::SerialPort() : serial_fd(-1) {}

// Destructor: ensure the serial port is closed.
SerialPort::~SerialPort() {
    closeSerial();
}

// Opens the serial port and configures it.
int SerialPort::openSerial(const char* port) {
    serial_fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        perror("Fejl ved åbning af serialport");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_fd, &tty) != 0) {
        perror("Fejl ved tcgetattr");
        close(serial_fd);
        serial_fd = -1;
        return -1;
    }
    
    // Set baud rate to 115200.
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                      // disable break processing
    tty.c_lflag = 0;                             // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                             // no remapping, no delays
    tty.c_cc[VMIN]  = 1;                         // Block until at least 1 byte is received
    tty.c_cc[VTIME] = 0;                         // No timeout (wait indefinitely)

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);        // shut off xon/xoff flow control
    tty.c_cflag |= (CLOCAL | CREAD);               // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);             // shut off parity
    tty.c_cflag &= ~CSTOPB;                        // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                       // no hardware flow control

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        perror("Fejl ved tcsetattr");
        close(serial_fd);
        serial_fd = -1;
        return -1;
    }
    return serial_fd;
}

// Writes a string to the serial port.
ssize_t SerialPort::writeData(const std::string& data) {
    if (serial_fd < 0) {
        std::cerr << "Serieporten er ikke åben!" << std::endl;
        return -1;
    }
    // Use the global ::write to avoid calling this method recursively.
    ssize_t bytesWritten = ::write(serial_fd, data.c_str(), data.size());
    if (bytesWritten < 0) {
        perror("Fejl ved skrivning til serialport");
    }
    return bytesWritten;
}

// Writes a single character by converting it to a string.
bool SerialPort::writeData(char c) {
    std::string s(1, c);
    return writeData(s) >= 0;
}

std::string SerialPort::readData() {
    if (serial_fd < 0) {
        std::cerr << "Serieporten er ikke åben!" << std::endl;
        return "";
    }
    char buffer[256];
    ssize_t bytesRead = ::read(serial_fd, buffer, sizeof(buffer));
    if (bytesRead < 0) {
        perror("Fejl ved læsning fra serialport");
        return "";
    }
    // Construct a string exactly from the received bytes.
    return std::string(buffer, bytesRead);
}


// Closes the serial port.
void SerialPort::closeSerial() {
    if (serial_fd >= 0) {
        close(serial_fd);
        serial_fd = -1;
    }
}

// Sends a vector of legal moves as a space-separated string followed by a newline.
int SerialPort::sendlegalmoves(const std::vector<std::string>& legalMoves) {
    std::string legalMovesString;
    for (const auto& move : legalMoves) {
        legalMovesString += move + " ";
    }
    if (!legalMovesString.empty() && legalMovesString.back() == ' ') {
        legalMovesString.pop_back();
    }
    legalMovesString += "\n";

    ssize_t bytesWritten = writeData(legalMovesString);
    if (bytesWritten < 0) {
        return 1;
    } else {
        std::cout << "Sendte legal moves string: " << legalMovesString;
    }
    return 0;
}

// Continuously reads from the serial port until a valid (>=4 characters) move is received.
std::string SerialPort::movefrompico() {
    while (true) {
        std::string move = readData();
        if (move.size() >= 4) {
            std::cout << "Data modtaget fra porten: " << move << std::endl;
            return move;
        }
        std::cout << "Ugyldig pakke registreret: " << move << std::endl;
    }
}

// Sends the command to close the gripper (writes '1').
bool SerialPort::closeGripper() {
    return writeData('1');
}

// Sends the command to open the gripper (writes '0').
bool SerialPort::openGripper() {
    return writeData('0');
}
