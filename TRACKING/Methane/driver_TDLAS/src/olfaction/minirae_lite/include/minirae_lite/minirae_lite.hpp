#include <stdlib.h>
#include <stdio.h>

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <olfaction_msgs/msg/gas_sensor.hpp>

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>


class MiniraeLite : public rclcpp::Node
{
public:

    MiniraeLite();
    void update();
    rclcpp::Rate m_rate;
    serial::Serial m_serial;

private:
    rclcpp::Publisher<olfaction_msgs::msg::GasSensor>::SharedPtr m_pub{ nullptr };
    std::string m_frame_id;
};