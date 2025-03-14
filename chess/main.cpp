#include "stockfishLinux.h"
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <vector>

int main() {
#ifndef _WIN32
    std::string stockfishPath = "./stockfish-ubuntu-x86-64-sse41-popcnt";
#endif

    std::cout << "Stockfish path: " << stockfishPath << std::endl;

    StockfishLinux engine = StockfishLinux(stockfishPath, 5);
    std::cout << "Stockfish engine created" << std::endl;

    engine.appendMovesMade("e2e4 e7e5");

    std::vector<std::string> legalMoves = engine.getLegalMoves();
    std::cout << "Legal moves: ";
    for (const std::string& move : legalMoves) {
        std::cout << move << " ";
    }
    std::cout << std::endl;

    std::string bestMove = engine.getBestMove();
    std::cout << "Best move: " << bestMove << std::endl;

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

    // Byg en string fra de lovlige træk og tilføj newline til sidst
    std::string legalMovesString;
    for (const std::string& move : legalMoves) {
        legalMovesString += move + " ";
    }
    if (!legalMovesString.empty() && legalMovesString.back() == ' ') {
        legalMovesString.pop_back();
    }
    legalMovesString += "\n";

    // Send legal moves string til Pico
    ssize_t bytesWritten = write(serial_fd, legalMovesString.c_str(), legalMovesString.size());
    if (bytesWritten < 0) {
        perror("Fejl ved skrivning til serialport");
    } else {
        std::cout << "Sendte legal moves string: " << legalMovesString;
    }

    // Luk serialporten og afslut programmet
    close(serial_fd);

    engine.~StockfishLinux();
    return 0;
}
