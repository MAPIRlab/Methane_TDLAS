// utility that just tries to identify if a device is the windsonic anemometer or not
// used for udev rules


#include <stdlib.h>
#include <stdio.h>

#include <string>
#include <serial/serial.h>

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>



bool verifyReading(std::string& data)
{
    std::vector<std::string> strings;
    boost::split(strings, data, boost::is_any_of(","));

    // CHECK FOR MESSAGE INTEGRITY
    if (strings.size() != 6)
        return false;

        return true;
}

int main(int argc, char** argv)
{
    serial::Serial my_serial;

    if(argc != 2)
    {
        printf("1");
        return 5;
    }
    // Get node parameters

    std::string port(argv[1]);
    int baudrate = 9600;

    try
    {
        my_serial.setPort(port);
        my_serial.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        my_serial.setTimeout(to);
        my_serial.open();
    }
    catch (const std::exception& e)
    {
        printf("1");
        return 2;
    }

    if (!my_serial.isOpen())
    {
        printf("1");
        return 3;
    }

    for(int i = 0; i < 5; i++)
    {
        std::string result;
        result = my_serial.readline(65536, "\r");
    
        if(verifyReading(result))
        {
            printf("0");
            return 0;
        }
        else
            continue;

        sleep(1);
    }
    printf("1");
    return 4;
}