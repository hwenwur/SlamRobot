#ifndef VELOCITY_READER_H
#define VELOCITY_READER_H

#define CACHE_LEN (30)

// 设为串口驱动输入缓冲区的大小，一般为4KB
#define BUFFER_LEN (4096)

#include "SerialFrame.h"
#include "robot_serial.h"

#include <thread>
#include <mutex>

class VelocityReader
{
public:
    VelocityReader(robotserial::Serial &mSerial);
    ~VelocityReader();
    bool seekHeader();
    bool lookupLatestFrame(SerialFrameTimestamped *frame);
    void startReadLoop();
    void stopReadLoop();

public:
    void readLoop();

private:
    robotserial::Serial &mSerial;

    SerialFrameTimestamped frameCache[CACHE_LEN];
    std::mutex frameCacheLock;
    int cacheStartPos;
    bool loopRunning;
    std::thread *readThread;
};

#endif