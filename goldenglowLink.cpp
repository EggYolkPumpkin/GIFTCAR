/*
 *  @author 潘骏羿
 *		@created 24.10.2025
 *
 * 适用于C1雷达+ESP32
 * 用于溥渊未来技术学院新生杯的雷达舵机马达驱动库
 * 接线要求:
 * 马达绿线11
 * 马达紫线12
 * 马达蓝线13
 * 舵机橙线14
 * 雷达黄线19
 * 雷达绿线20
 */
#include "goldenglowLink.h"

float lidarData[360]; // 映射数据量
void initialize()
{
  Serial.begin(115200);
#pragma region 舵机
  pinMode(SERVO, OUTPUT); // 设定舵机接口为输出接口
  digitalWrite(SERVO, LOW);
#pragma endregion
#pragma region 马达
  pinMode(MOTOR_PWM, OUTPUT); // 设置各引脚为输出模式
  pinMode(MOTOR_A, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);
#pragma endregion
}

void lidarLoop()
{
  lidarData[0] = 100;
#pragma region 雷达
  static rpLidar lidar(&Serial2, 460800);
  Serial2.setRxBufferSize(5000);
  lidar.resetDevice();              // 重启雷达
  lidar.setAngleOfInterest(0, 359); // 测量范围
  bool ret = lidar.start();
  //*
  if (ret)
    printf("rplidar C1 started correctly!!!\r\n");
  else
    printf("error starting rplidar C1\r\n");
  //*/
#pragma endregion
  while (true)
  {
    // printf("lidar loop\n");
    static int latestIndex = 0;            // 用于记录上一次调用本函数时扫描到的最新角度
    int count = lidar.readMeasurePoints(); // 读取新数据
    if (count == 0)
    {
      delay(1);
      continue;
    }
    else
    {
      for (int i = 0; i < count; i++)
      {
        float angle = (lidar.DataBuffer[i].angle_high * 128 + lidar.DataBuffer[i].angle_low / 2) / 64.0;
        float distance = (lidar.DataBuffer[i].distance_high * 256 + lidar.DataBuffer[i].distance_low) / 4.0;
        int quality = lidar.DataBuffer[i].quality / 4;
        if (quality == 0 || distance == 0)
          continue;
        int index = ((int)roundf(angle)) % 360;
        if (index < 0 || index >= 360)
          continue;
        // 采用循环列表存储距离数据
        // 从latestIndex+1到index-1的角度都被漏扫了,应该归零!!!
        for (int offset = 1; offset < (index - latestIndex + 360) % 360; offset++)
        {
          lidarData[(offset + latestIndex) % 360] = 0;
        }
        latestIndex = index;
        lidarData[index] = distance;
      }
    }
    // 喂狗
    delay(1);
  }
}

void getLidarData(float *buffer)
{
  for (int i = 0; i < 360; i++)
    buffer[i] = lidarData[i];
}

void operateServo(float steering)
{
  if (steering < 0)
    steering = 0;
  if (steering > 180)
    steering = 180;
  int pulsewidth = (steering * 11) + 500; // 将角度转化为500-2480的脉宽值
  digitalWrite(SERVO, HIGH);              // 将舵机接口电平至高
  delayMicroseconds(pulsewidth);          // 延时脉宽值的微秒数
  digitalWrite(SERVO, LOW);               // 将舵机接口电平至低
  delayMicroseconds(2480 - pulsewidth);
}

void operateMotor(float speed)
{
  if (speed > 1)
    speed = 1;
  if (speed < -1)
    speed = -1;
  if (speed >= 0)
  {
    digitalWrite(MOTOR_A, HIGH);
    digitalWrite(MOTOR_B, LOW);
    analogWrite(MOTOR_PWM, (int)(255 * speed));
  }
  else
  {
    digitalWrite(MOTOR_A, LOW);
    digitalWrite(MOTOR_B, HIGH);
    analogWrite(MOTOR_PWM, (int)(255 * -speed));
  }
}