#include "debug.h"

void debugReadDistance()
{
    float data[360];
    getLidarData(data);
    Serial.printf("front:%.1f;\tright:%.1f;\tback:%.1f;\tleft:%.1f\n", data[0], data[90], data[180], data[270]);
    int tmp = 0;
    for (int i = 0; i < 360; i++)
        if (data[i] != 0)
            tmp++;
    Serial.printf("count:%d\n", tmp);
}

void debugOperateServo()
{
    // 让前轮在80°到100°之间反复摆动:
    for (int i = 80; i < 100; i++)
    {
        // 向舵机写入PWM值
        operateServo(i);
        delay(50);
    }
    for (int i = 100; i > 80; i--)
    {
        // 向舵机写入PWM值
        operateServo(i);
        delay(50);
    }
}

void debugOperateMotor()
{
    for (int i = -100; i <= 100; i++)
    {
        operateMotor(i / 100.0f);
        delay(10);
    }
    for (int i = 100; i >= -100; i--)
    {
        operateMotor(i / 100.0f);
        delay(10);
    }
}
