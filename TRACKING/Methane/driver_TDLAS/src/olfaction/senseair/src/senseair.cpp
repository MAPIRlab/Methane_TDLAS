#include <senseair/senseair.hpp>

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	std::shared_ptr<SenseAir> node = std::make_shared<SenseAir>();

	node->run();

	return (0);
}

SenseAir::SenseAir() : Node("Senseair") {}

void SenseAir::run()
{
	serial::Serial my_serial;

	std::string port = declare_parameter<std::string>("port", "/dev/ttyUSB0");
	int baudrate = declare_parameter("baud", 9600);
	std::string frame_id = declare_parameter<std::string>("frame_id", "/s8_link");
	std::string topic = declare_parameter<std::string>("topic", "/co2_reading");

	RCLCPP_INFO(get_logger(), "Initializing module at port:%s:%u on frame reference:%s", port.c_str(), baudrate, frame_id.c_str());

	auto measurement_pub = create_publisher<olfaction_msgs::msg::GasSensor>(topic, 10);

	// Open serial port
	try
	{
		my_serial.setFlowcontrol(serial::flowcontrol_none);
		my_serial.setParity(serial::parity_none);
		my_serial.setStopbits(serial::stopbits_one);
		my_serial.setPort(port);
		my_serial.setBaudrate(baudrate);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		my_serial.setTimeout(to);
		my_serial.open();
	}
	catch (serial::IOException& e)
	{
		RCLCPP_ERROR(get_logger(), "Senseair s8 -- Failed to open serial port!");
	}

	if (my_serial.isOpen())
		RCLCPP_INFO(get_logger(), "Serial port initialized.");
	else
		return;

	static const uint8_t readCommand[] = { 0xFE, 0x04, 0x00, 0x03, 0x00, 0x01, 0xD5, 0xC5 };

	// Loop
	auto shared_this = shared_from_this();
	rclcpp::Rate loop_rate(0.5);
	while (rclcpp::ok())
	{
		rclcpp::spin_some(shared_this);

		my_serial.flush();
		// Request the sensor a new  reading
		my_serial.write(readCommand, 8);

		if (my_serial.available())
		{
			olfaction_msgs::msg::GasSensor sensor_msg;
			std::string result;
			// result = my_serial.read(my_serial.available());
			result = my_serial.read(7);

			// string to ppm
			unsigned char firstByte = result[3];
			unsigned char secondByte = result[4];
			unsigned int ppm = (firstByte << 8) + secondByte;

			RCLCPP_INFO(get_logger(), "Read ppm: %u", ppm);

			// Create msg and publish
			sensor_msg.header.stamp = now();
			sensor_msg.header.frame_id = frame_id.c_str();

			sensor_msg.raw_units = sensor_msg.UNITS_PPM;
			sensor_msg.raw_air = 0.0;
			sensor_msg.raw = ppm;

			measurement_pub->publish(sensor_msg);
		}

		loop_rate.sleep();
	}

	my_serial.close();
}