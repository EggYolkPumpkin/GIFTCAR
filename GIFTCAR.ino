#include "core.h"
void thread1(void *_);
void thread2(void *_);

void setup()
{
  initCore();
  // 线程1,用于读取雷达数据(这雷达太难伺候了,我用了一整个国庆假期才把这东西写出来)
  xTaskCreatePinnedToCore(thread1, "thr1", 16384, NULL, 2, NULL, 0);
  // 线程2,用于数据处理和决策
  xTaskCreatePinnedToCore(thread2, "thr2", 16384, NULL, 2, NULL, 1);
  // 绑定函数,名字,栈大小,NULL,优先级,NULL,核心
}

void loop()
{
  // 空着别动
}

// for thread1&2:请保证
// 1.必须为死循环
// 2.循环里必须要有delay()函数,不然会导致忘记喂狗导致程序假死
// 感兴趣请自行搜索,关键词watch dog
void thread1(void *_)
{
  // 这个函数自带delay();
  lidarLoop();
}
void thread2(void *_)
{
  delay(1000);
  while (true)
  {
    loopCore();
    delay(1);
  }
}
