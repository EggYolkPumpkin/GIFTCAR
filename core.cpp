#include "core.h"
#include "debug.h"
/// @brief 获取雷达数据
void scan();
/// @brief 决策
void decide();
/// @brief 开车
void move();
void initCore()
{
    // 初始化代码请放在这里
    initialize();
    Serial.println("Initialized");
}

void loopCore()
{
    // 重复执行的代码请放在这里
    // debugReadDistance();
    // debugOperateServo();
    // debugOperateMotor();
    scan();
    decide();
    move();
}





























int mid = 90;
int angle_offset = -5;                                                // adjust the servo midpoint
float data[360];                                                      // 雷达距离数组，序号即角度（°），内容即距离（mm）
float p1_ang, p1_dis, p2_ang, p2_dis, p3_ang, p3_dis, p4_ang, p4_dis; // 用于搜寻雷达检测范围内的可信点
float x_m, steering;                                                  // x_m：小车距离赛道中心的偏移量（mm），steering：小车转向角度（°）
float X1, Y1, X2, Y2, W1, W2;                                         // 左侧检测范围内选取两点的坐标及角度
float X3, Y3, X4, Y4, W3, W4;                                         // 右侧检测范围内选取两点的坐标及角度
int go_back = 0;                                                      // 倒车标志
int last_go_back = 0;
float ang1 = 0, ang2 = 0, ang0 = 0; // 左右侧赛道边对应的航向角，ang0为平均值

void scan()
{
    getLidarData(data);
}

void decide()
{
    // 遍历data中每个角度的距离数据
    for (int i = 0; i < 360; i++)
    {
        float dis = data[i];
        if (i > 45 && i < 90 && dis != 0 && dis < 800) // 检测右侧赛道边情况
        {
            if (p4_ang == 0)
            {
                p4_dis = dis;
                p4_ang = i;
                W4 = i * -1; // 顺时针（雷达旋转方向）转逆时针（目标坐标系）
                X4 = p4_dis * cos(W4 * PI / 180);
                Y4 = p4_dis * sin(W4 * PI / 180); // 取得右侧赛道边近点坐标
            }
            if (i > p3_ang)
            {
                p3_dis = dis;
                p3_ang = i;
                W3 = i * -1;
                X3 = p3_dis * cos(W3 * PI / 180);
                Y3 = p3_dis * sin(W3 * PI / 180); // 取得右侧赛道边远点坐标
            }
        }
        else if (i > 270 && i < 315 && dis != 0 && dis < 800)
        {
            if (p1_ang < 270)
            {
                p1_dis = dis;
                p1_ang = i;
                W1 = 360 - i;
                X1 = p1_dis * cos(W1 * PI / 180);
                Y1 = p1_dis * sin(W1 * PI / 180); // 取得左侧赛道边近点坐标
            }
            if (i > p2_ang)
            {
                p2_dis = dis;
                p2_ang = i;
                W2 = 360 - i;
                X2 = p2_dis * cos(W2 * PI / 180);
                Y2 = p2_dis * sin(W2 * PI / 180); // 取得左侧赛道边远点坐标
            }
        }
        else if (i == 359)
        {
            p1_ang = 0;
            p1_dis = 0;
            p2_ang = 0;
            p2_dis = 0;
            p3_ang = 0;
            p3_dis = 0;
            p4_ang = 0;
            p4_dis = 0;
        }
        // 检测是否无法通过小半径弯道，离墙太近，需要倒车
        if (dis <= 300 && dis != 0 && ((i > 350 && i < 359) || (i > 0 && i < 10)))
        {
            Serial.println("back");
            go_back = 1;
            break;
        }
        else if (dis >= 400 && ((i > 350 && i < 359) || (i > 0 && i < 10)))
        {
            go_back = 0;
        }
        data[i] = 0; // 处理完一个点将其置0，以免迟迟不更新导致污染数据
    }
}

void move()
{
    ang1 = atan2((Y2 - Y1), (X2 - X1)); // 左侧赛道边角度值
    ang2 = atan2((Y4 - Y3), (X4 - X3)); // 右侧赛道边角度值
    ang1 = (ang1 / PI * 180);
    ang2 = (ang2 / PI * 180);

    if (fabs(ang1) > fabs(ang2))
        ang0 = ang1;
    else
        ang0 = ang2; // 以角度较大的一侧作为转向依据
    Serial.print("ang0:");
    Serial.println(ang0);
    Serial.print(Y1);
    Serial.print("  ");
    Serial.println(Y3);

    x_m = (Y1 + Y3) / 2;                 // 计算小车偏离赛道中心的位移量
    steering = (1.5 * ang0 + 0.2 * x_m); // 加权计算小车实际转向角

    // 当遇到大转向角弯道，无法一次通过时，需要先倒车，同时前轮打反方向
    if (go_back == 1)
    {
        steering = steering * -1;
        operateServo(steering + mid + angle_offset);
        if (last_go_back != go_back)
        {
            operateMotor(0);
            delay(500);
        }
    }
    else
        operateServo(steering + mid + angle_offset);

    // 控制速度，最大1m/s，最小0.6m/s
    // float speed = 0.8 + (12 - abs(steering)) / 12 * 0.2;
    float speed = 1;
    if (go_back == 1)
        operateMotor(speed * -1);
    else
        operateMotor(speed);
    last_go_back = go_back;
}