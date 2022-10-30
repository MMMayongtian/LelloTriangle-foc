#include "RGB.h"
const int threshold_top = 40;   //触摸阈值
const int single_count[3] = {10,10,10};   //单击时间   实际时间为20*10 = 200ms
const int long_count[3] = {80,80,80};   //长按时间    实际时间为80*10 = 800ms
unsigned long touch_last_time;
int touch_count[3] = {0,0,0}; //持续触摸计数
int touch_touched[3] = {0,0,0};   //单击,长按判断  单击值为1，长按值为2  没点击为0
bool touch_STATE[3] = {1, 1, 1}; // 定义按键触发对象状态变量初始值为true默认开启 T2 T3 T4

int rgb_flag = 1;
int rgb_modle = 8;//有几种RGB效果就写几
//触摸感应处理
void touchAttach(int touchID, uint8_t touchPin) {
  int touchread = touchRead(touchPin);
  if ( touchread <= threshold_top ) { //达到触发值的计数
    //delay(38);  // 0.038秒
    touch_count[touchID]++; //持续触摸计数
  } 
  else
  {
  if ( touch_count[touchID] >= single_count[touchID] && touch_count[touchID] < long_count[touchID])
    touch_touched[touchID] = 1;//持续触摸时间达到单击
  else if(touch_count[touchID] >= long_count[touchID])
    touch_touched[touchID] = 2;
  else
      touch_touched[touchID]= 0;
  touch_count[touchID] = 0;  //持续触摸计数清零
  }
}
