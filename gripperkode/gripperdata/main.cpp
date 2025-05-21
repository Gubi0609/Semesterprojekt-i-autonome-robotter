
#include <iostream>
#include <vector>
#include <string>
#include <boost/asio.hpp>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
using boost::asio::serial_port_base;

struct CloseEvent {
    std::vector<uint16_t> samples;
};

int main() {
    const std::string port_name = "/dev/ttyACM0";

    // --- Open the serial port ---
    boost::asio::io_service io;
    boost::asio::serial_port serial(io, port_name);
    serial.set_option(serial_port_base::baud_rate(115200));
    serial.set_option(serial_port_base::character_size(8));
    serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));

    std::cout << "Listening for dump on " << port_name << " …\n";

    std::vector<CloseEvent> events;
    boost::asio::streambuf buf;
    std::string line;
    bool in_dump = false;
    size_t samples_to_read = 0;

    // --- Read until we see "=== END OF DUMP ===" ---
    while (true) {
        boost::asio::read_until(serial, buf, "\n");
        std::istream is(&buf);
        std::getline(is, line);

        if (line.rfind("=== DUMPING", 0) == 0) {
            events.clear();
            in_dump = true;
            continue;
        }
        if (!in_dump) continue;

        if (line.rfind("Event", 0) == 0) {
            events.emplace_back();
            auto pos = line.find("samples=");
            samples_to_read = (pos != std::string::npos)
                ? std::stoul(line.substr(pos + 8))
                : 0;
            continue;
        }
        if (samples_to_read > 0) {
            uint16_t v = static_cast<uint16_t>(std::stoul(line));
            events.back().samples.push_back(v);
            if (events.back().samples.size() >= samples_to_read)
                samples_to_read = 0;
            continue;
        }
        if (line.rfind("=== END OF DUMP ===", 0) == 0) {
            break;
        }
    }

    // --- Convert ADC values to voltages and plot each event ---
    constexpr double VREF = 3.3;            // Reference voltage in volts
    constexpr double SCALE = VREF / 4095.0; // Scale factor for 12-bit ADC

    for (size_t i = 0; i < events.size(); ++i) {
        const auto &ev = events[i];
        std::vector<double> x(ev.samples.size()), y(ev.samples.size());
        for (size_t j = 0; j < ev.samples.size(); ++j) {
            x[j] = j*4;
            // Convert ADC value to current:
            y[j] = ((ev.samples[j] * SCALE) / 30 / 0.5)-0.02;
        }
        plt::named_plot("Event " + std::to_string(i+1), x, y);
    }
    plt::xlabel("millisekunder");
    plt::ylabel("Current (A)");
    plt::legend();
    plt::title("Gripper Close Events");
    plt::show();

    return 0;
}


/*g++ main.cpp -o gripper_plot \
    -std=c++17 \
    -I./include \
    $(python3-config --cflags) \
    -I$(python3 -c "import numpy; print(numpy.get_include())") \
    -lboost_system \
    -pthread \
    $(python3-config --ldflags --embed) \
    -Wno-deprecated-declarations
*/