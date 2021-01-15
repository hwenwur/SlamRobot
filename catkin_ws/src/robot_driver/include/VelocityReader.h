#ifndef VELOCITY_READER_H
#define VELOCITY_READER_H

#define CACHE_LEN (30)

#include "SerialFrame.h"
#include "robot_serial.h"

#include <thread>

class VelocityReader
{
public:
    VelocityReader(robotserial::Serial &mSerial);
    ~VelocityReader();
    bool seekHeader();
    bool lookupLatestFrame(SerialFrameTimestamped &frame);
    void startReadLoop();
    void stopReadLoop();

public:
    void readLoop();

private:
    robotserial::Serial &mSerial;

    SerialFrameTimestamped frameCache[CACHE_LEN];
    int cacheStartPos;
    bool loopRunning;
    std::thread *readThread;
};

#endif