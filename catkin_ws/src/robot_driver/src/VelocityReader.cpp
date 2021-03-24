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

bool VelocityReader::lookupLatestFrame(SerialFrameTimestamped *frame)
{
    std::lock_guard<std::mutex> l(frameCacheLock);
    if (cacheStartPos < 0)
    {
        std::cerr << "cacheStartPos < 0; not have any cache.\n";
        return false;
    }
    else
    {
        *frame = frameCache[cacheStartPos];
        std::cerr << "Velocity(" << frame->frame.vx << ", " << frame->frame.vy << ", " << frame->frame.omega << ");\n";
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
    static char buff[BUFFER_LEN];
    cacheStartPos = -1;
    SerialFrameTimestamped tmp;
    int count, available;
    int i;

    loopRunning = true;
    while (loopRunning)
    {
        available = mSerial.availableBytes();
        if (available < sizeof(SerialFrame) + 2)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        available = available - (available % (sizeof(SerialFrame) + 2));
        available = available > BUFFER_LEN ? BUFFER_LEN : available;

        count = mSerial.read(buff, available);
        if (count < 0)
        {
            std::cerr << "Read serial failed: " << strerror(errno) << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        else if (count == 0)
        {
            // std::cerr << "Waiting for serial data(count = 0)...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        else if (count < (sizeof(SerialFrame) + 2))
        {
            std::cerr << "Waiting serial(count < 14)...\n";
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

                    std::lock_guard<std::mutex> l(frameCacheLock);
                    cacheStartPos = (cacheStartPos + 1) % CACHE_LEN;
                    frameCache[cacheStartPos] = tmp;
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