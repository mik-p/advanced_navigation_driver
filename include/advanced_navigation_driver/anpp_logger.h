/**
 * MIT License
 *
 * Copyright (c) 2017 an-scott
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <fstream>
#include <an_packet_protocol.h>

#pragma once

namespace an_driver
{

class ANPPLogger
{

public:
    bool open(const std::string &filename)
    {
        logFile.open(filename.c_str(), std::ios::out | std::ios::binary);
        return logFile.is_open();
    }

    bool write(const an_packet_t &packet)
    {
        if (!logFile.is_open())
        {
            return false;
        }
        logFile.write((char *)&packet.header, sizeof(uint8_t) * AN_PACKET_HEADER_SIZE);
        logFile.write((char *)&packet.data, packet.length * sizeof(uint8_t));
        return true;
    }

private:
    std::ofstream logFile;
};

} // namespace an_driver
