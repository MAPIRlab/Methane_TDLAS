#include <windsonic/windsonic.hpp>

// MAIN
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<WindSonic> node = std::make_shared<WindSonic>();
    node->run();

    return (0);
}

WindSonic::WindSonic() : Node("Windsonic")
{
}

void WindSonic::run()
{
    serial::Serial my_serial;

    // Get node parameters

    std::string port = declare_parameter<std::string>("port", "/dev/ttyUSB1");
    int baudrate = declare_parameter("baud", 9600);
    frame_id = declare_parameter<std::string>("frame_id", "windsonic_link");
    std::string topic = declare_parameter<std::string>("topic", "/wind");

    RCLCPP_INFO(get_logger(), "Initializing module at port:%s:%u on frame reference:%s", port.c_str(), baudrate, frame_id.c_str());

    wind_pub = create_publisher<olfaction_msgs::msg::Anemometer>(topic, 10);
    windmarker_pub = create_publisher<visualization_msgs::msg::Marker>("Windvector_display", 100);

    // Open serial port
    try
    {
        my_serial.setPort(port);
        my_serial.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        my_serial.setTimeout(to);
        my_serial.open();
    }
    catch (serial::IOException& e)
    {
        RCLCPP_ERROR(get_logger(), "WindSonic -- Failed to open serial port!");
        raise(SIGTRAP);
    }

    if (my_serial.isOpen())
        RCLCPP_INFO(get_logger(), "Serial port initialized.");
    else
        return;

    // Read Loop
    rclcpp::Rate loop_rate(5);
    auto shared_this = shared_from_this();
    displayTime = now();

    while (rclcpp::ok())
    {
        rclcpp::spin_some(shared_this);

        if (my_serial.available())
        {
            std::string result;
            result = my_serial.readline(65536, "\r");

            read_anemometer(result);
        }
        loop_rate.sleep();
    }

    my_serial.close();
}

float get_average_wind_direction(std::vector<float> const& v)
{
    // Average of wind direction, avoiding the problems of +/- pi angles.
    float x = 0.0, y = 0.0;
    for (std::vector<float>::const_iterator i = v.begin(); i != v.end(); ++i)
    {
        x += cos(*i);
        y += sin(*i);
    }
    float average_angle = atan2(y, x);

    return average_angle;
}

float get_average_vector(std::vector<float> const& v)
{
    size_t length = v.size();
    float sum = 0.0;
    for (std::vector<float>::const_iterator i = v.begin(); i != v.end(); ++i)
        sum += *i;

    return sum / length;
}

WindSonic::WindMeasurement WindSonic::parseWindMeasurement(std::string& data)
{
    // WindSonic mode (default is Polar continuous)
    // format = <STX>Q, dir, speed, units, status, <ETX>\r
    // Q      -> Sensor identifier
    // dir    -> 3 digits (0-359)
    // speed  -> %3.2f
    // units  -> M (m/s), K (km/h)
    // status -> 00 = ok, else is an Error Code

    WindMeasurement wind;

    // SEPARATE MESSAGE INTO SUB-STRINGS
    std::vector<std::string> strings;
    boost::split(strings, data, boost::is_any_of(","));

    // CHECK FOR MESSAGE INTEGRITY
    if (strings.size() != 6)
    {
        RCLCPP_ERROR(get_logger(), "[windsonic_node] Message could not be parsed:");
        for (int i = 0; i < strings.size(); i++)
        {
            RCLCPP_ERROR(get_logger(), "%s", strings[i].c_str());
        }
        RCLCPP_ERROR(get_logger(), "-----");
        wind.id = 'x';
        return wind;
    }

    // PARSE

    // 0. Q
    strings[0].erase(0, 1);                     // Delete 0x02 (STX)
    sscanf(strings[0].c_str(), "%c", &wind.id); // ID (usually='Q')

    // 1. DIR 3 digits (0-359), empty if no wind
    if (strings[1].length() < 3)
    {
        wind.direction = -1;
    }
    else
    {
        sscanf(strings[1].c_str(), "%3u", &wind.direction);
    }

    // 2. SPEED %3.2f
    if (strings[2].length() > 6)
    {
        wind.speed = -1.0;
    }
    else
    {
        sscanf(strings[2].c_str(), "%f", &wind.speed);
    }

    // 3. UNITS
    if (strings[3].length() != 1)
    {
        wind.units = '?';
    }
    else
    {
        sscanf(strings[3].c_str(), "%c", &wind.units);
        if (wind.units != 'M' && wind.units != 'K')
        {
            wind.units = '?';
        }
    }

    // 4. STATUS
    if (strings[4].length() < 1)
    {
        wind.status = -1;
    }
    else
    {
        sscanf(strings[4].c_str(), "%u", &wind.status);
    }

    return wind;
}

void WindSonic::read_anemometer(std::string& data)
{
    // GET WIND MEASUREMENT
    WindSonic::WindMeasurement wind = parseWindMeasurement(data);

    if (wind.id == 'x')
    {
        // if there was an error parsing, ignore the message instead of closing the node
        return;
    }

    // PROCESS DIRECTION -> If none, no wind detected
    if (wind.direction < 0)
    {
        wind.direction = 0;
        wind.speed = 0.0;
    }

    // PROCESS SPEED -> Check if valid, and convert to m/s
    if (wind.speed < 0)
    {
        wind.direction = 0;
        wind.speed = 0.0;
        RCLCPP_ERROR(get_logger(), "[windsonic] Reported invalid wind speed");
    }
    else
    {
        if (wind.units == 'M')
        {
            RCLCPP_DEBUG(get_logger(), "[windsonic] Wind speed in M/S");
        }
        else if (wind.units == 'K')
        {
            RCLCPP_DEBUG(get_logger(), "[windsonic] Wind speed in Km/h");
            wind.speed *= 1000.0 / 3600.0;
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "[windsonic] Wind speed reported in invalid units");
            wind.direction = 0;
            wind.speed = 0.0;
        }
    }

    // PROCESS STAUTS
    if (wind.status != 0)
    {
        RCLCPP_WARN(get_logger(), "[windsonic] Satus = %d, wind-reading might be unrealiable", wind.status);
    }

    // VERBOSE
    RCLCPP_INFO(get_logger(), "[windsonic] Speed: %f\tDirection: %d\tStatus: %d", wind.speed, wind.direction, wind.status);

    // CREATE OLFACTION_MSGS ANEMOMETER MESSAGE
    olfaction_msgs::msg::Anemometer wind_msg;

    wind_msg.header.stamp = now();
    wind_msg.header.frame_id = frame_id.c_str();
    wind_msg.sensor_label = "windSonic";
    wind_msg.wind_speed = wind.speed;
    wind_msg.wind_direction = angles::from_degrees(-wind.direction) + M_PI - M_PI / 4; // deg to rad

    wind_pub->publish(wind_msg);

    windS.push_back(wind_msg.wind_speed);
    windD.push_back(angles::normalize_angle(wind_msg.wind_direction));

    if ((now() - displayTime).seconds() >= 1)
    {
        float average_wind_direction = get_average_wind_direction(windD);
        float average_wind_speed = get_average_vector(windS);

        visualization_msgs::msg::Marker wind_point_inv;
        wind_point_inv.header.frame_id = frame_id.c_str();
        wind_point_inv.action = visualization_msgs::msg::Marker::ADD;
        wind_point_inv.ns = "measured_wind_inverted";
        wind_point_inv.type = visualization_msgs::msg::Marker::ARROW;

        wind_point_inv.header.stamp = now();
        wind_point_inv.points.clear();
        wind_point_inv.id = 1; // unique identifier for each arrow
        wind_point_inv.pose.position.x = 0;
        wind_point_inv.pose.position.y = 0;
        wind_point_inv.pose.position.z = 0.0;
        wind_point_inv.pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, average_wind_direction));
        wind_point_inv.scale.x = average_wind_speed; // arrow lenght
        wind_point_inv.scale.y = 0.1;                // arrow width
        wind_point_inv.scale.z = 0.1;                // arrow height
        wind_point_inv.color.r = 0.0;
        wind_point_inv.color.g = 1.0;
        wind_point_inv.color.b = 0.0;
        wind_point_inv.color.a = 1.0;
        windmarker_pub->publish(wind_point_inv);
        displayTime = now();
        windD.clear();
        windD.clear();
    }
}
