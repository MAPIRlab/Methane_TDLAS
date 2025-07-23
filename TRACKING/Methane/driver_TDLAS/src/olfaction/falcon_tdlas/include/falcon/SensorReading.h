#pragma once
#include <stdint.h>
#include <array>
#include <falcon/MSGCodes.h>

struct SingleReading
{
    int PPMxM;
    float reflectionStrength; //no units given
    float absorptionStrength; //no units given
    uint64_t timestamp;

    SingleReading()
        :PPMxM(0), reflectionStrength(0), absorptionStrength(0)
    {}
};

struct InMessage
{
    bool valid;

    int average_PPMxM; //over the entire 0.5s interval
    std::array<SingleReading, 5> readings; //0.1s intervals

    InMessage(const std::string& msg)
    {
        constexpr char DELIMITER = ';';
        std::stringstream ss(msg);
        std::string buffer;

        std::getline(ss, buffer, DELIMITER); // FWD 1
        valid = buffer.back() == '1';

        std::getline(ss, buffer, DELIMITER);
        average_PPMxM = std::stoi(buffer);

        for (int i = 0; i < 5; i++)
        {
            std::getline(ss, buffer, DELIMITER);
            readings[i].PPMxM = std::stoi(buffer);

            std::getline(ss, buffer, DELIMITER);
            readings[i].reflectionStrength = std::stof(buffer);

            std::getline(ss, buffer, DELIMITER);
            readings[i].absorptionStrength = std::stof(buffer);

            std::getline(ss, buffer, DELIMITER);
            readings[i].timestamp = std::stoul(buffer);
        }
    }

    InMessage() : valid(false), average_PPMxM(0) {}
};