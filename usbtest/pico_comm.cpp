#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include <cstdlib>

int main() {
    // Change this to match your Pico's serial device (e.g., /dev/ttyACM0)
    const char* port = "/dev/ttyACM0";

    // Open the serial port for read/write, no controlling terminal, synchronous I/O.
    int serial_fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        perror("Error opening serial port");
        return EXIT_FAILURE;
    }

    // Set up serial port parameters.
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_fd, &tty) != 0) {
        perror("Error from tcgetattr");
        close(serial_fd);
        return EXIT_FAILURE;
    }

    // Set input and output baud rate to 115200.
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // Configure the serial port: 8-bit characters, no parity, one stop bit.
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit characters.
    tty.c_iflag &= ~IGNBRK;                      // Disable break processing.
    tty.c_lflag = 0;                             // No signaling chars, no echo, no canonical processing.
    tty.c_oflag = 0;                             // No remapping, no delays.
    tty.c_cc[VMIN]  = 1;                         // Read blocks until at least 1 char is received.
    tty.c_cc[VTIME] = 1;                         // 0.1 seconds read timeout.

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);        // Shut off xon/xoff control.
    tty.c_cflag |= (CLOCAL | CREAD);               // Ignore modem controls, enable reading.
    tty.c_cflag &= ~(PARENB | PARODD);             // Shut off parity.
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;                       // No hardware flow control.

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        close(serial_fd);
        return EXIT_FAILURE;
    }

    std::cout << "Connected to " << port << " at 115200 baud." << std::endl;
    std::cout << "Type commands (e.g., \"on\" or \"off\") and press Enter." << std::endl;

    // Buffer for serial I/O.
    char buf[256];

    // Main loop: multiplex input from STDIN and the serial port.
    while (true) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(serial_fd, &readfds);
        FD_SET(STDIN_FILENO, &readfds);

        // Get the highest file descriptor value for select.
        int max_fd = (serial_fd > STDIN_FILENO) ? serial_fd : STDIN_FILENO;

        // Wait indefinitely for input on either file descriptor.
        int rv = select(max_fd + 1, &readfds, nullptr, nullptr, nullptr);
        if (rv < 0) {
            perror("select");
            break;
        }

        // Check if there is incoming data from the Pico.
        if (FD_ISSET(serial_fd, &readfds)) {
            ssize_t n = read(serial_fd, buf, sizeof(buf) - 1);
            if (n > 0) {
                buf[n] = '\0';
                std::cout << "[Pico] " << buf;
            }
        }

        // Check if user input is available on the PC's STDIN.
        if (FD_ISSET(STDIN_FILENO, &readfds)) {
            std::string line;
            if (std::getline(std::cin, line)) {
                // Append a newline, as the Pico expects newline-terminated commands.
                line += "\n";
                ssize_t written = write(serial_fd, line.c_str(), line.length());
                if (written < 0) {
                    perror("write");
                }
            } else {
                // If getline fails, break the loop.
                break;
            }
        }
    }

    close(serial_fd);
    return EXIT_SUCCESS;
}
