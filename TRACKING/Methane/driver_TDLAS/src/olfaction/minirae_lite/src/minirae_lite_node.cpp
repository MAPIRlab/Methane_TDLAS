#include <minirae_lite/minirae_lite.hpp>

// MAIN
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<MiniraeLite> node = std::make_shared<MiniraeLite>();

    // Loop
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        node->update();
        node->m_rate.sleep();
    }

    node->m_serial.close();
    return (0);
}

MiniraeLite::MiniraeLite() : Node("Minirae_lite"), m_rate(20)
{
    // Get node parameters
    std::string port = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    int baudrate = declare_parameter("baud", 9600);
    m_frame_id = declare_parameter<std::string>("frame_id", "minirae_lite_link");
    std::string topic_name = declare_parameter<std::string>("topic_name", "/PID_reading");

    RCLCPP_INFO(get_logger(), "Initializing module at port:%s:%u on frame reference:%s", port.c_str(), baudrate, m_frame_id.c_str());
    m_pub = create_publisher<olfaction_msgs::msg::GasSensor>(topic_name, 10);
    RCLCPP_INFO(get_logger(), "Publishing on topic '%s'", m_pub->get_topic_name());

    // Open serial port
    try
    {
        m_serial.setPort(port);
        m_serial.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        m_serial.setTimeout(to);
        m_serial.open();
    }
    catch (serial::IOException& e)
    {
        RCLCPP_ERROR(get_logger(), "MiniRAE_lite -- Failed to open serial port!");
        raise(SIGTRAP);
    }

    if (m_serial.isOpen())
        RCLCPP_INFO(get_logger(), "Serial port initialized.");
}

void MiniraeLite::update()
{
    try
    {
        m_serial.flush();
        // Request the PID a new  reading
        m_serial.write("R");
        if (m_serial.available())
        {
            olfaction_msgs::msg::GasSensor sensor_msg;
            std::string result;
            // result = m_serial.read(m_serial.available());
            result = m_serial.readline(500, "\r");
            // RCLCPP_INFO(get_logger(), "Read string:" << result);
            // string to ppm
            float ppm = atof(result.c_str()) / 1000;

            // RCLCPP_INFO(get_logger(), "Read ppm: %.2f", ppm);

            // Create msg and publish
            sensor_msg.header.stamp = now();
            sensor_msg.header.frame_id = m_frame_id.c_str();

            sensor_msg.technology = sensor_msg.TECH_PID;
            sensor_msg.manufacturer = sensor_msg.MANU_RAE;
            sensor_msg.mpn = sensor_msg.MPN_MINIRAELITE;
            sensor_msg.raw_units = sensor_msg.UNITS_PPM;
            sensor_msg.raw_air = 0.0;
            sensor_msg.raw = ppm;

            m_pub->publish(sensor_msg);
        }
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "Exception: '%s'", e.what());
    }
}
