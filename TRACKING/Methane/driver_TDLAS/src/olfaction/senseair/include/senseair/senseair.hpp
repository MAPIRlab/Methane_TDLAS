#include <stdlib.h>
#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include <olfaction_msgs/msg/gas_sensor.hpp>

class SenseAir : public rclcpp::Node
{
public:
    SenseAir();
    void run();
private:
};