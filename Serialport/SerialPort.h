#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <string>
#include <vector>
#include <unistd.h>  // For ssize_t

class SerialPort {
public:
    // Constructor and destructor.
    SerialPort();
    ~SerialPort();

    // Opens the serial port using the provided port name.
    // Returns the file descriptor if successful, otherwise -1.
    int openSerial(const char* port);

    // Writes the given string data to the serial port.
    // Returns the number of bytes written or -1 on error.
    ssize_t writeData(const std::string& data);

    // Writes a single character to the serial port.
    // Returns true on success, false on error.
    bool writeData(char c);

    // Reads data from the serial port and returns it as a string.
    std::string readData();

    // Closes the serial port.
    void closeSerial();

    // Sends a vector of legal moves as a space-separated string followed by a newline.
    // Returns 0 on success, 1 on failure.
    int sendLegalMoves(const std::vector<std::string>& legalMoves);

    // Continuously reads from the serial port until a string of at least 4 characters is received.
    std::string moveFromPico();

    // Sends a command to close the gripper (writes '1').
    bool closeGripper();

    // Sends a command to open the gripper (writes '0').
    bool openGripper();

private:
    int serial_fd;  // File descriptor for the serial port.
};

#endif // SERIALPORT_H
