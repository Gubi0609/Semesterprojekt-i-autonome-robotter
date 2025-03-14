#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <string>
#include <sstream>
#include <vector>

int main() {
    // Ændr porten, hvis nødvendigt (fx "/dev/ttyACM0" eller "/dev/ttyUSB0")
    const char* port = "/dev/ttyACM0";

    // Åbn serialporten til læsning og skrivning.
    int serial_fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        perror("Fejl ved åbning af serialport");
        return EXIT_FAILURE;
    }

    // Konfigurer serialporten.
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_fd, &tty) != 0) {
        perror("Fejl ved tcgetattr");
        close(serial_fd);
        return EXIT_FAILURE;
    }
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit bytes
    tty.c_iflag &= ~IGNBRK;                      // deaktiver break-behandling
    tty.c_lflag = 0;                           // ingen kanonisk tilstand, ingen echo osv.
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 1;                       // blokér indtil mindst 1 byte modtages
    tty.c_cc[VTIME] = 1;                       // 0,1 sekunders timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);      // sluk for software flow control
    tty.c_cflag |= (CLOCAL | CREAD);             // ignorer modemkontrol, tillad læsning
    tty.c_cflag &= ~(PARENB | PARODD);           // ingen paritet
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;                     // ingen hardware flow control

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        perror("Fejl ved tcsetattr");
        close(serial_fd);
        return EXIT_FAILURE;
    }

    std::cout << "Lytter på " << port << " med 115200 baud." << std::endl;
    char buf[256];
    while (true) {
        // Læs indtil vi modtager en linje (afsluttet med '\n')
        int i = 0;
        memset(buf, 0, sizeof(buf));
        while (true) {
            int n = read(serial_fd, &buf[i], 1);
            if (n > 0) {
                if (buf[i] == '\n') break;
                i++;
                if (i >= (int)sizeof(buf) - 1) break;
            }
        }
        buf[i] = '\0';
        std::string message(buf);
        std::cout << "Modtaget fra Pico: " << message << std::endl;
        
        // Når beskeden starter med "MOVE:", opret og send vectoren af moves.
        if (message.find("MOVE:") == 0) {
            // Opret en vector med moves
            std::vector<std::string> moves = {"a2b3", "a4a2", "b2b4"};
            // Kombinér dem til én streng med et protokolhoved "MOVES:".
            std::ostringstream oss;
            oss << "MOVES:";
            for (size_t j = 0; j < moves.size(); j++) {
                oss << moves[j];
                if (j != moves.size() - 1) {
                    oss << ",";
                }
            }
            oss << "\n";
            std::string response = oss.str();
            // Send svaret til Picoen.
            ssize_t bytes_written = write(serial_fd, response.c_str(), response.size());
            if (bytes_written < 0) {
                perror("Fejl ved skrivning til serialport");
            } else {
                std::cout << "Sendte respons: " << response;
            }
        }
    }

    close(serial_fd);
    return EXIT_SUCCESS;
}
