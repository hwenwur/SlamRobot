#include "VelocityReader.h"

#include <iostream>
#include <chrono>
#include <thread>

VelocityReader::VelocityReader(robotserial::Serial &mSerial) : mSerial(mSerial),
                                                               cacheStartPos(-1),
                                                               loopRunning(false),
                                                               readThread(0)
{
    mSerial.setBlocking(false);
    mSerial.open();
    if (!mSerial.isOpen())
    {
        std::cerr << "robotserial::Serial not open or open failed.\n";
    }
}

VelocityReader::~VelocityReader()
{
    mSerial.close();
    stopReadLoop();
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
            std::cerr << "Read serial failed.\n";
            break;
        }
        else if (r == 0)
        {
            std::cerr << "Waiting for serial data...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        else
        {
            if (curr == '\n' && last == '\r')
            {
                std::cerr << "Find header of Frame\n";
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
    // const size_t BUFFER_LEN = (sizeof(SerialFrame) + 2) * 10;
    // char buff[BUFFER_LEN];
    // int count = -1;
    // count = mSerial.read(buff, BUFFER_LEN);
    // if (count < 0)
    // {
    //     std::cerr << "Error: " << strerror(errno) << "\n";
    //     return false;
    // }
    // else if (count == 0)
    // {
    //     // waiting for data
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }
    // else if (count < 14)
    // {
    //     std::cerr << "Error: count(" << count << ") < 14\n";
    //     return false;
    // }
    // else
    // {
    //     int i;
    //     for (i = count - 1; i > 0; i--)
    //     {
    //         if (buff[i] == '\n' && buff[i - 1] == '\r' && (i - 1 - sizeof(SerialFrame) >= 0))
    //         {
    //             SerialFrame *f = (SerialFrame *)(buff + (i - 1 - sizeof(SerialFrame)));
    //             frame.frame = *f;
    //             frame.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
    //                                   std::chrono::system_clock::now().time_since_epoch())
    //                                   .count();
    //             std::cerr << "Velocity(" << f->vx << ", " << f->vy << ", " << f->omega << ");\n";
    //             return true;
    //         }
    //     }
    //     if (i == 0)
    //     {
    //         std::cerr << "Failed to find header\n";
    //         return false;
    //     }
    // }
    // return false;

    if (cacheStartPos < 0)
    {
        std::cerr << "cacheStartPos < 0; not have any cache.\n";
        return false;
    }
    else
    {
        frame = frameCache[cacheStartPos];
        std::cerr << "Velocity(" << frame.frame.vx << ", " << frame.frame.vy << ", " << frame.frame.omega << ");\n";
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
    if (readThread && readThread->joinable())
    {
        readThread->join();
        std::cerr << "Readloop exited\n";
    }
    if (readThread)
    {
        delete readThread;
        readThread = 0;
    }
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
            std::cerr << "Waiting for serial data(count = 0)...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        else if (count < (sizeof(SerialFrame) + 2))
        {
            std::cerr << "Waiting serial(count < 14)...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
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
                    break;
                }
            }
            if (i == 0)
            {
                std::cerr << "Faild to find header(count > 14)\n";
            }
        }
    }
}