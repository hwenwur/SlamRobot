#include "VelocityReader.h"
#include "robot_serial.h"
#include <iostream>
#include <unistd.h>


int main()
{
  robotserial::Serial mSerial("/dev/ttyUSB0", 115200);
  VelocityReader reader(mSerial);
  reader.startReadLoop();
  SerialFrameTimestamped vel;
  bool r;
  while(true)
  {
    r = reader.lookupLatestFrame(&vel);
    if(r)
      printf("%.2f, %.2f, %.2f, %lu\n", vel.frame.vx, vel.frame.vy, vel.frame.omega, vel.timestamp / 1000);
    else
      printf("Error\n");
    sleep(1);
  }
  reader.stopReadLoop();
  return 0;
}
