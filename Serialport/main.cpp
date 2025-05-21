#include <iostream>
#include <string>
#include "SerialPort.h"

int main() {
    const char* port = "/dev/ttyACM0";  // Adjust this to your serial port device.
    SerialPort serial;

    if (serial.openSerial(port) < 0) {
        return 1;
    }

    std::string command;
    std::cout << "Serial port opened successfully." << std::endl;
    
    while (true) {
        std::cout << "Enter command (\"opengripper\", \"closegripper\", \"read\", \"legal\", \"move\", \"exit\"):" << std::endl;
        std::cout << "> ";
        std::getline(std::cin, command);

        if (command == "exit") {
            break;
        } else if (command == "closegripper") {
            serial.closeGripper();
        } else if (command == "opengripper") {
            serial.openGripper();
        } else if (command == "stopgripper") {
            serial.stopGripper();
        } else if (command == "read") {
            std::cout << "Received: " << serial.readData() << std::endl;
        } else if (command == "legal") {
            // Example legal moves.
            std::vector<std::string> moves = {"e2e4", "d2d4", "g1f3"};
            serial.sendlegalmoves(moves);
        } else if (command == "move") {
            std::string move = serial.movefrompico();
            std::cout << "Move from pico: " << move << std::endl;
        } else {
            std::cout << "Unknown command!" << std::endl;
        }
    }

    serial.closeSerial();
    return 0;
}
