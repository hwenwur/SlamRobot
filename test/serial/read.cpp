#include "robot_serial.h"
#include <thread>
#include <chrono>

typedef struct
{
    float vx;    // x 轴线速度
    float vy;    // y 轴线速度
    float omega; // z 轴角速度
} SerialFrame;


int main()
{
    robotserial::Serial mSerial("/dev/stm32", 115200);
    mSerial.setBlocking(true);
    mSerial.open();
    char buff[256];
    int count;
    while((count=mSerial.read(buff, 256)) >= 0)
    {
		printf("count = %d\n", count);
		if(count == 14)
		{
			SerialFrame *f = (SerialFrame*)&buff;
			printf("%.2f, %.2f, %.2f\n", f->vx, f->vy, f->omega);
		}
        for(int i = 0; i < count; i++)
        {
//            putchar(buff[i]);
        }
	std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }
    printf("END: %s\n", strerror(errno));
    mSerial.close();
    return 0;
}

