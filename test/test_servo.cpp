#include <iostream>
#include "SCServo.h"
#include <thread>
#include <chrono>

SMS_STS sm_st;

int main(int argc, char ** argv)
{
    const char* port = "/dev/ttyUSB0";
    uint8_t mode = 0;

    if (argc > 1) {
        // 1st optional argument is the serial port
        port = argv[1];
    }
    if (argc > 2) {
        // 2nd optional argument is the
        // control mode: (0=position, 1=velocity, 2=effort/PWM)
        mode = std::atoi(argv[2]);
        mode = std::min<uint8_t>(mode, 2);
        mode = std::max<uint8_t>(mode, 0);
    }
    if (argc > 3) {
        std::cout << "Invalid argument [" << argv[2] << "]. "
                  << "Expect at most two arguments [[serial_port] [mode]]"
                  << std::endl;
        return -1;
    }
    std::cout << "serial:" << port << std::endl;

    if(!sm_st.begin(1000000, port)){
        std::cout << "Failed to init sms/sts motor!" << std::endl;
        return 0;
    }

    // Set Mode
    std::cout << "Mode: " << static_cast<int>(mode) << std::endl;
    for(uint8_t id = 1; id <= 6; id++) {
        if (!sm_st.Mode(id, mode)) {
            std::cout << "Failed to set Mode on servo "
            << static_cast<int>(id) << std::endl;
            return -1;
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Read data from all servos
    for(uint8_t id = 1; id <= 6; id++) {
        std::cout << "\nReading servo "
                  << static_cast<int>(id) << "..." << std::endl;
        
        if (sm_st.FeedBack(id) != -1) {
            // Position (0-4095)
            int pos = sm_st.ReadPos(id);
            
            // Speed (-32767 - 32767)
            int speed = sm_st.ReadSpeed(id);
            
            // Load (-1000 - 1000)
            int load = sm_st.ReadLoad(id);
            
            // Voltage (V)
            double voltage = sm_st.ReadVoltage(id) / 10.0;
            
            // Temperature (°C)
            int temp = sm_st.ReadTemper(id);
            
            // Move status (0=stopped, 1=moving)
            int move = sm_st.ReadMove(id);
            
            std::cout << "  Position: " << pos << " (0-4095)" << std::endl;
            std::cout << "  Speed: " << speed << std::endl;
            std::cout << "  Load: " << load << std::endl;
            std::cout << "  Voltage: " << voltage << "V" << std::endl;
            std::cout << "  Temperature: " << temp << "°C" << std::endl;
            std::cout << "  Moving: " << (move ? "yes" : "no") << std::endl;
        } else {
            std::cout << "  Failed to read feedback from servo "
                      << static_cast<int>(id) << std::endl;
        }
    }

    sm_st.end();
    return 0;
}
