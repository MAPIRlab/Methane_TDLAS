#pragma once

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <olfaction_msgs/msg/tdlas.hpp>
#include <falcon/OutMessage.h> 
#include <falcon/SensorReading.h>

class Falcon : public rclcpp::Node
{
public:
    Falcon();
    ~Falcon() = default;

    void run();

private:
    rclcpp::Publisher<olfaction_msgs::msg::TDLAS>::SharedPtr m_publisher{ nullptr };

    struct Settings
    {
        float frequency;
        std::string port;
        std::string topic;
        std::string frame_id;
        bool verbose;
    };
    Settings m_settings;

    bool initSerial(serial::Serial& serial);
    bool doHandShake(serial::Serial& serial);
    InMessage getReading(serial::Serial& serial);

    void publish(const InMessage& msg);
};
