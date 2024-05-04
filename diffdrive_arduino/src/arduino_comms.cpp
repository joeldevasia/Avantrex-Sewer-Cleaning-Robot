#include "diffdrive_arduino/arduino_comms.h"
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
using namespace std::chrono_literals;


void ArduinoComms::readEncoderValues(int &val_1, int &val_2)
{
    // std::string response = sendMsg("e\r");

    // std::string delimiter = " ";
    // size_t del_pos = response.find(delimiter);
    // std::string token_1 = response.substr(0, del_pos);
    // std::string token_2 = response.substr(del_pos + delimiter.length());

    // val_1 = std::atoi(token_1.c_str());
    // val_2 = std::atoi(token_2.c_str());
}

void ArduinoComms::setMotorValues(int val_1, int val_2)
{
    // std::stringstream ss;
    // ss << "m " << val_1 << " " << val_2 << "\r";
    // sendMsg(ss.str(), false);
}