#include <falcon/falcon.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<Falcon> falcon = std::make_shared<Falcon>();

    falcon->run();
    return 0;
}

Falcon::Falcon() : Node("Falcon_TDLAS")
{
    m_settings.frequency = declare_parameter<float>("frequency", 2);
    m_settings.port = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    m_settings.topic = declare_parameter<std::string>("topic", "/falcon/reading");
    m_settings.frame_id = declare_parameter<std::string>("frame_id", "falcon_link");
    m_settings.verbose = declare_parameter<bool>("verbose", false);

    m_publisher = create_publisher<olfaction_msgs::msg::TDLAS>(m_settings.topic, 10);
}

void Falcon::run()
{
    serial::Serial serial;
    if (!initSerial(serial))
        return;

    rclcpp::Rate rate(m_settings.frequency);

    while (!doHandShake(serial) && rclcpp::ok())
        rate.sleep();

    auto shared_this = shared_from_this();

    std::string bytesBuffer;
    while (rclcpp::ok())
    {
        rclcpp::spin_some(shared_this);

        InMessage reading = getReading(serial);

        if (reading.valid)
            publish(reading);

        rate.sleep();
    }

    serial.close();
}

bool Falcon::initSerial(serial::Serial& serial)
{
    serial.setBaudrate(19200);
    serial.setStopbits(serial::stopbits_one);
    serial.setParity(serial::parity_none);
    serial.setBytesize(serial::eightbits);
    serial.setPort(m_settings.port);
    serial.setTimeout(serial::Timeout::max(), 250, 0, 250, 0);

    try
    {
        serial.open();
    }
    catch (std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "%s", e.what());
    }

    if (!serial.isOpen())
    {
        RCLCPP_ERROR(get_logger(), "Failed to open serial port\n");
        return false;
    }
    return true;
}

bool Falcon::doHandShake(serial::Serial& serial)
{

    // version message
    {
        const std::array<uint8_t, 3> VER = { 0x56, 0x45, 0x52 };
        OutMessage message;

        message << MSGCodes::BEGIN_MESSAGE << MSGCodes::ETC << MSGCodes::SEPARATOR
            << VER << MSGCodes::QUERY
            << MSGCodes::END_MESSAGE;

        message << message.BCC();

        serial.write(message.data);

        if (!serial.waitReadable())
        {
            RCLCPP_INFO(get_logger(), "Timeout - No response during handshake");
            return false;
        }

        auto read = serial.read(256);
        if (read.size() == 0 || read.at(0) != MSGCodes::ACK)
        {
            RCLCPP_ERROR(get_logger(), "Error during handshake! Received message did not contain an ACK: %s\n", read.c_str());
            return false;
        }
    }

    {
        const std::array<uint8_t, 3> ALL = { 0x41, 0x4C, 0x4C };

        serial.write(&MSGCodes::ACK, 1);
        OutMessage message;
        message << MSGCodes::BEGIN_MESSAGE << MSGCodes::CMN << MSGCodes::SEPARATOR
            << ALL << MSGCodes::QUERY << MSGCodes::END_MESSAGE;

        message << message.BCC();
        message << std::vector<uint8_t>(message.data); // duplicate the message without the ACK (for some reason)
        serial.write(message.data);

        if (!serial.waitReadable())
        {
            RCLCPP_INFO(get_logger(), "Timeout - No response during handshake");
            return false;
        }

        auto read = serial.read(512);

        if (read.size() == 0 || read.at(0) != MSGCodes::ACK)
        {
            RCLCPP_ERROR(get_logger(), "Error during handshake! Received message did not contain an ACK: %s\n", read.c_str());
            return false;
        }

        serial.write(&MSGCodes::ACK, 1);
    }

    return true;
}

InMessage Falcon::getReading(serial::Serial& serial)
{
    OutMessage message;
    message << MSGCodes::ACK << MSGCodes::BEGIN_MESSAGE
        << MSGCodes::ETC << MSGCodes::SEPARATOR << MSGCodes::FWD
        << MSGCodes::QUERY << MSGCodes::END_MESSAGE;

    message << message.BCC();

    serial.write(message.data);

    if (!serial.waitReadable())
    {
        RCLCPP_INFO(get_logger(), "Timeout - No response to reading request");
        return {};
    }

    auto read = serial.read(256);
    if (read.size() == 0 || read.at(0) != MSGCodes::ACK)
    {
        RCLCPP_ERROR(get_logger(), "Error getting a sensor reading! Received message did not contain an ACK: %s\n", read.c_str());
        return {};
    }

    InMessage reading;
    try
    {
        reading = InMessage(read);
    }
    catch (std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "Exception: %s received while trying to parse sensor reading: %s\n", e.what(), read.c_str());
    }

    if (m_settings.verbose)
    {
        std::cout << "[FALCON_TDLAS] ";
        std::cout << (reading.valid ? "valid" : "ERROR") << "\n"
            << "PPMxM: " << reading.average_PPMxM << "\n"
            << "\n";
        for (auto it = reading.readings.begin(); it != reading.readings.end(); it++)
        {
            std::cout << "\tPPMxM: " << it->PPMxM << "\n"
                << "\tReflectivity: " << it->reflectionStrength << "\n"
                << "\tAbsorptivity: " << it->absorptionStrength << "\n"
                << "\tTimestamp: " << it->timestamp << "\n";
        }
    }

    return reading;
}

uint8_t OutMessage::BCC()
{
    uint8_t running = 0;

    uint8_t last_read = 0;
    int i = 0;
    bool started = false;
    while (i < data.size() && last_read != MSGCodes::END_MESSAGE)
    {
        if (last_read == MSGCodes::BEGIN_MESSAGE)
            started = true;

        if (started)
            running ^= data[i];

        last_read = data[i];
        i++;
    }
    return running;
}

void Falcon::publish(const InMessage& reading)
{
    olfaction_msgs::msg::TDLAS msg;

    // Header
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = m_settings.frame_id;

    // Raw samples
    for (const auto& raw_s : reading.readings)
    {
        msg.ppmxm.push_back(raw_s.PPMxM);                            // ppm x meter
        msg.reflection_strength.push_back(raw_s.reflectionStrength); // no units given
        msg.absorption_strength.push_back(raw_s.absorptionStrength); // no units given
    }

    // Average values
    msg.average_ppmxm = reading.average_PPMxM;
    msg.average_reflection_strength = std::accumulate(msg.reflection_strength.begin(), msg.reflection_strength.end(), 0) / std::size(reading.readings);
    msg.average_absorption_strength = std::accumulate(msg.absorption_strength.begin(), msg.absorption_strength.end(), 0) / std::size(reading.readings);

    m_publisher->publish(msg);
}
