#include <iostream>
#include "SCServo.h"
#include <thread>
#include <chrono>

SMS_STS sm_st;

void restore_mode(uint8_t servo_id) {
    if (!sm_st.Mode(servo_id, 0)) {
        std::cout << "Failed to restore mode for servo "
                  << static_cast<int>(servo_id) << std::endl;
    }
}

int main(int argc, char ** argv)
{
    const char* port = "/dev/ttyUSB0";

    uint8_t servo_id = 1;
    uint8_t mode = 2;
    int16_t pwm = 0;

    if (argc > 1) {
        // 1st optional argument is the serial port
        port = argv[1];
    }
    if (argc > 2) {
        // 2nd optional argument is the pwm in [-1000, 1000]
        pwm = std::atoi(argv[2]);
        pwm = std::min<int16_t>(pwm, 1000);
        pwm = std::max<int16_t>(pwm, -1000);
    }
    if (argc > 3) {
        std::cout << "Invalid argument [" << argv[2] << "]. "
                  << "Expect at most two arguments [[serial_port] [pwm]]"
                  << std::endl;
        return -1;
    }
    std::cout << "serial:" << port << std::endl;

    if (!sm_st.begin(1000000, port)) {
        std::cout << "Failed to init sms/sts motor!" << std::endl;
        return 0;
    }

    std::cout << "\nWriting to servo "
              << static_cast<int>(servo_id) << "..." << std::endl;
  
    // Set Mode 2 (PWM)
    std::cout << "Mode: " << static_cast<int>(mode) << std::endl;
    if (!sm_st.Mode(servo_id, mode)) {
        std::cout << "Failed to set Mode on servo "
                  << static_cast<int>(servo_id) << std::endl;
        restore_mode(servo_id);
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Write PWM
    std::cout << "WritePwm: " << pwm << std::endl;
    if (!sm_st.WritePwm(servo_id, pwm)) {
        std::cout << "Failed to WritePwm to servo "
                  << static_cast<int>(servo_id) << std::endl;
        restore_mode(servo_id);
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Read data from all servos
    for (uint8_t id = servo_id; id <= servo_id; ++id) {
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

    restore_mode(servo_id);
    sm_st.end();
    return 0;
}
