#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

int main() {
    // Ændr porten hvis nødvendigt (fx "/dev/ttyACM0" eller "/dev/ttyUSB0")
    const char* port = "/dev/ttyACM0";

    // Åbn serialporten for læsning og skrivning
    int serial_fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        perror("Fejl ved åbning af serialport");
        return 1;
    }

    // Konfigurer serialporten
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_fd, &tty) != 0) {
        perror("Fejl ved tcgetattr");
        close(serial_fd);
        return 1;
    }
    
    // Sæt baudrate til 115200
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // Konfigurer porten: 8 data bits, ingen paritet, 1 stop bit, ingen flow control
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;             // Ingen kanonisk mode, ingen echo
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 1;         // Bloker indtil mindst 1 byte modtages
    tty.c_cc[VTIME] = 1;         // Timeout på 0,1 sekund

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;     // Ingen hardware flow control

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        perror("Fejl ved tcsetattr");
        close(serial_fd);
        return 1;
    }

    // Definer vector string, der skal sendes (afsluttet med newline)
    std::string vectorString = "a2b4 b2b4 c1c3\n";

    // Send vector string til Picoen
    ssize_t bytesWritten = write(serial_fd, vectorString.c_str(), vectorString.size());
    if (bytesWritten < 0) {
        perror("Fejl ved skrivning til serialport");
    } else {
        std::cout << "Sendte vector string: " << vectorString;
    }

    // Luk serialporten og afslut programmet
    close(serial_fd);
    return 0;
}
