#ifndef SERIAL_FRAME_H
#define SERIAL_FRAME_H

typedef struct
{
    float vx;    // x 轴线速度
    float vy;    // y 轴线速度
    float omega; // z 轴角速度
} SerialFrame;

typedef struct
{
    SerialFrame frame;
    unsigned long timestamp; // 时间戳
} SerialFrameTimestamped;

#endif
