#include <string>
#include <iostream>

int main() {
    int number = 42;
    std::string str = std::to_string(number);
    // Use the string as needed
    std::cout << "Converted string: " << str << std::endl;
    return 0;
}
