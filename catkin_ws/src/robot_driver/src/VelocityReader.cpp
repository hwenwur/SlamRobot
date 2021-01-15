#include "VelocityReader.h"

#include <iostream>
#include <chrono>
#include <thread>

VelocityReader::VelocityReader(robotserial::Serial &mSerial) : mSerial(mSerial), cacheStartPos(-1)
{
    mSerial.setBlocking(true);
    mSerial.open();
    if (!mSerial.isOpen())
    {
        std::cerr << "robotserial::Serial not open or open failed.\n";
    }
    seekHeader();
}

VelocityReader::~VelocityReader()
{
    mSerial.close();
    if (loopRunning)
    {
        loopRunning = false;
        readThread->join();
    }
    delete readThread;
}

bool VelocityReader::seekHeader()
{
    char curr, last;
    int max_try = 1024;
    int r;
    while (--max_try >= 0)
    {
        r = mSerial.read(&curr, 1);
        if (r < 0)
        {
            std::cerr << "Read serial failed.";
            break;
        }
        else if (r == 0)
        {
            std::cerr << "Waiting for serial data...";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        else
        {
            if (curr == '\n' && last == '\r')
            {
                // ready = true;
                return true;
            }
            last = curr;
        }
    }
    std::cerr << "Can not find header of Frame.\n";
    return false;
}

bool VelocityReader::lookupLatestFrame(SerialFrameTimestamped &frame)
{
    if (cacheStartPos < 0)
    {
        return false;
    }
    else
    {
        frame = frameCache[cacheStartPos];
        return true;
    }
}
void VelocityReader::startReadLoop()
{
    readThread = new std::thread(&VelocityReader::readLoop, this);
}

void VelocityReader::stopReadLoop()
{
    loopRunning = false;
    readThread->join();
}

void VelocityReader::readLoop()
{
    const size_t BUFFER_LEN = (sizeof(SerialFrame) + 2) * 10;
    char buff[BUFFER_LEN];
    cacheStartPos = -1;
    SerialFrameTimestamped tmp;
    int count;
    int i;

    loopRunning = true;
    while (loopRunning)
    {
        count = mSerial.read(buff, BUFFER_LEN);
        if (count < 0)
        {
            std::cerr << "Read serial failed: " << strerror(errno) << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        else if (count == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        else if (count < (sizeof(SerialFrame) + 2))
        {
            continue;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        else
        {
            for (i = count - 1; i > 0; i--)
            {
                if (buff[i] == '\n' && buff[i - 1] == '\r' && (i - 1 - sizeof(SerialFrame) >= 0))
                {
                    memcpy(&tmp, buff + (i - 1 - sizeof(SerialFrame)), sizeof(SerialFrame));
                    tmp.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::system_clock::now().time_since_epoch())
                                        .count();

                    frameCache[(cacheStartPos + 1) % CACHE_LEN] = tmp;
                    cacheStartPos = (cacheStartPos + 1) % CACHE_LEN;
                }
            }
        }
    }
}