#include <stdlib.h>
#include <stdio.h>

#include <string>

#include <serial/serial.h>
#include <unistd.h>
#include <algorithm>

int print(int v)
{
    printf("%d", v);
    return v;
}

bool is_valid_reading(const std::string& s)
{
    if(s.empty())
        return false;

    if(s.back() != '\r')
        return false;

    std::stringstream ss(s);
    long num;
    ss >> num;
    
    if(num == 0 && s[0] != '0')
        return false;
    return true;
}


int main(int argc, char** argv)
{
    if(argc!=2)
        return print(1);

    std::string port = argv[1];
    int baudrate = 9600;

    serial::Serial m_serial;

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
        return print(2);
    }

    m_serial.flush();
    m_serial.write("R");
    for(int i = 0; i< 5; i++)
    {
        std::string result;
        result = m_serial.readline(500, "\r");
        // printf("result: %s\n", result.c_str()); 
        
        if(!is_valid_reading(result))
            continue;
    
        float ppm = atof(result.c_str()) / 1000;
    
        if(ppm >= 0 && ppm <1000)
            return print(0);
        
        sleep(1);
    } 
    return print(4);
}