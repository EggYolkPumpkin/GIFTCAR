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
#ifndef goldenglow_link_h
#define goldenglow_link_h
#include "rpLidar.h"
#include "rpLidarTypes.h"

#ifndef SERVO
#define SERVO 14 //舵机口
#endif

#ifndef MOTOR
#define MOTOR
#define MOTOR_PWM 11  //小车电机速度控制口
#define MOTOR_A 13    //小车电机方向控制口
#define MOTOR_B 12    //小车电机方向控制口
#endif

/// @brief 初始化小车
void initialize();

/// @brief 获取雷达的数据并存在buffer数组里,内容为从当前时间~一圈前的有效数据
/// @param buffer buffer[i]表示雷达测得的距离(单位mm),无效数据为0
void getLidarData(float* buffer);

/// @brief 读取雷达信息(死循环,请使用多线程技术)
void lidarLoop();

/// @brief 操作舵机
/// @param steering 舵机角度,单位为°
void operateServo(float steering);

/// @brief 操作马达
/// @param speed 功率,取值[-1,1],建议取值的绝对值不小于0.6,不然开不动
void operateMotor(float speed);
#endif