#include "robot_serial.h"
#include <thread>
#include <chrono>

typedef struct
{
    float vx;    
    float vy;    
    float omega;
} SerialFrame;


int main(int argc, char *argv[])
{
    robotserial::Serial mSerial("/dev/stm32", 115200);
    mSerial.open();
    SerialFrame f = {0, 0, 6.28};
    while(true)
    {
        mSerial.writeAll(&f, sizeof(SerialFrame));
        mSerial.writeAll("\r\n", 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    mSerial.close();
    return 0;
}

