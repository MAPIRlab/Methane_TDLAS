#include <stdlib.h>
#include <stdio.h>

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <olfaction_msgs/msg/anemometer.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>
#include <angles/angles.h>

class WindSonic : public rclcpp::Node
{
public:
    WindSonic();
    void run();

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr windmarker_pub;
    rclcpp::Publisher<olfaction_msgs::msg::Anemometer>::SharedPtr wind_pub;
    std::string frame_id;
    std::vector<float> windS;
    std::vector<float> windD;
    rclcpp::Time displayTime;

    struct WindMeasurement
    {
        char id;
        int direction;
        float speed;
        char units;
        int status;
    };

    WindMeasurement parseWindMeasurement(std::string& data);
    void read_anemometer(std::string& data);

};