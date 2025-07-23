#pragma once

#include <array>

class MSGCodes
{
public:

    inline static constexpr uint8_t ACK = 0x06;
    inline static constexpr uint8_t BEGIN_MESSAGE = 0x02;
    inline static constexpr uint8_t END_MESSAGE = 0x03;
    inline static constexpr uint8_t SEPARATOR = 0x3A;

    inline static constexpr std::array<uint8_t, 3> QUERY = { 0x20, 0x3F, 0x3B };
    inline static constexpr std::array<uint8_t, 3> FWD = { 0x46, 0x57, 0x44 };

    //message types
    inline static constexpr std::array<uint8_t, 3> ETC = { 0x45, 0x54, 0x43 };
    inline static constexpr std::array<uint8_t, 3> CMN = { 0x43, 0x4D, 0x4E };
    inline static constexpr std::array<uint8_t, 3> RTC = { 0x52, 0x54, 0x43 };

};