#pragma once
#include <vector>
#include <array>

class OutMessage
{
public:
    std::vector<uint8_t> data;

    OutMessage& operator<<(const uint8_t& byte)
    {
        data.push_back(byte);
        return *this;
    }

    template<size_t N>
    OutMessage& operator<<(const std::array<uint8_t, N>& bytes)
    {
        for (auto it = bytes.begin(); it != bytes.end(); it++)
            data.push_back(*it);
        return *this;
    }

    OutMessage& operator<<(const std::vector<uint8_t>& bytes)
    {
        for (auto it = bytes.begin(); it != bytes.end(); it++)
            data.push_back(*it);
        return *this;
    }

    uint8_t BCC();
};