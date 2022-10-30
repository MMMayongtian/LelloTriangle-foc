  /**
arduino开发环境-灯哥开源FOChttps://gitee.com/ream_d/Deng-s-foc-controller
FOC引脚32, 33, 25
AS5600霍尔传感器 SDA-23 SCL-5  MPU6050六轴传感器 SDA-19 SCL-18
本程序平衡控制为速度控制，LQR参数使用K3和K4
在wifi上位机窗口中输入：TA+角度，就可以修改平衡角度
比如让平衡角度为90度，则输入：TA90，并且会存入eeprom的位置0中 注：wifi发送命令不能过快，因为每次都会保存进eeprom
在使用自己的电机时，请一定记得修改默认极对数，即 BLDCMotor(7) 中的值，设置为自己的极对数数字，磁铁数量/2
程序默认设置的供电电压为 12V,用其他电压供电请记得修改 voltage_power_supply , voltage_limit 变量中的值
默认PID针对的电机是 2715 ，使用自己的电机需要修改PID参数，才能实现更好效果
 */
#include <SimpleFOC.h>
#include "Command.h"
#include <WiFi.h>
#include <AsyncUDP.h> //引用以使用异步UDP
#include <ArduinoOTA.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include "EEPROM.h"
#include "tourch.h"
/* ----ESP32 IO SET---- */
#define ACTIVE_PIN 4  //状态灯
#define BAT_VOLTAGE_SENSE_PIN 34  //电池电压检测ADC，如果旧版PCB无电压检测电路，则注释掉此行
const double R1_VOLTAGE = 62000; //62K
const double R2_VOLTAGE = 10000; //10K
const double min_voltage  = 9.5;  //电池检测最低电压
double bat_voltage;
unsigned long voltage_last_time;
/* ----IMU Data---- */
Kalman kalmanZ;
#define gyroZ_OFF -0.19
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
bool stable = 0 , battery_low = 0;
uint32_t last_unstable_time;
uint32_t last_stable_time;

double gyroZangle; // Angle calculate using the gyro only
double compAngleZ; // Calculated angle using a complementary filter
double kalAngleZ;  // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
/* ----FOC Data---- */

// driver instance
const char *ServerName = "ESP32-Reuleaux-RGB";
char mac_tmp[6];
const char *ssid = mac_tmp;
const char *password = "";
bool wifi_on_off = 0;
bool wifi_flag = 0;
AsyncUDP udp;                     //创建UDP对象
unsigned int localUdpPort = 2333; //本地端口号
void wifi_print(char * s,double num);

/* ----FOC Data---- */
double acc2rotation(double x, double y);
float constrainAngle(float x);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Ctwo = TwoWire(1);
LowPassFilter lpf_throttle{0.00};

//倒立摆参数
float LQR_K3_1 = 8.4;   //摇摆到平衡
float LQR_K3_2 = 2.1;   //
float LQR_K3_3 = 2.1; //

float LQR_K4_1 = 2.4;   //平衡到稳定
float LQR_K4_2 = 1.5;   //
float LQR_K4_3 = 1.42; //

//电机参数
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25);
float target_velocity = 0;  //目标速度
float target_angle = 90;  //平衡角度 例如TA89.3 设置平衡角度89.3
float target_voltage = 0; //目标电压
float swing_up_voltage = 1.4; //摇摆电压 左右摇摆的电压，越大越快到平衡态，但是过大会翻过头
float swing_up_angle = 12;  //摇摆角度 离平衡角度还有几度时候，切换到自平衡控制
float v_i_1 = 25; //非稳态速度环I
float v_p_1 = 1.8;  //非稳态速度环P
float v_i_2 = 10; //稳态速度环I
float v_p_2 = 0.3;  //稳态速度环P

//命令设置
Command comm;
bool Motor_enable_flag = 0;
int test_flag = 0;
void do_TA(char* cmd) { comm.scalar(&target_angle, cmd);EEPROM.writeFloat(0, target_angle); }
void do_SV(char* cmd) { comm.scalar(&swing_up_voltage, cmd); EEPROM.writeFloat(4, swing_up_voltage); }
void do_SA(char* cmd) { comm.scalar(&swing_up_angle, cmd);EEPROM.writeFloat(8, swing_up_angle); }
void do_START(char* cmd) {  wifi_flag = !wifi_flag; }
void do_MOTOR(char* cmd)
{  
  if(Motor_enable_flag)
    motor.enable();
  else 
    motor.disable();
  Motor_enable_flag = !Motor_enable_flag;
}
void do_TVQ(char* cmd)
{
  if(test_flag == 1)
    test_flag = 0;
  else
  {
    motor.controller = MotionControlType::torque;
    test_flag = 1;
  }
}
void do_TVV(char* cmd)
{
  if(test_flag == 2)
    test_flag = 0;
  else
  {
    motor.controller = MotionControlType::velocity;
    test_flag = 2;
  }
}
void do_VV(char* cmd) { comm.scalar(&target_velocity, cmd); }
void do_VQ(char* cmd) { comm.scalar(&target_voltage, cmd); }
void do_vp1(char* cmd) { comm.scalar(&v_p_1, cmd); EEPROM.writeFloat(12, v_p_1);}
void do_vi1(char* cmd) { comm.scalar(&v_i_1, cmd);EEPROM.writeFloat(16, v_i_1); }
void do_vp2(char* cmd) { comm.scalar(&v_p_2, cmd); EEPROM.writeFloat(20, v_p_2);}
void do_vi2(char* cmd) { comm.scalar(&v_i_2, cmd);EEPROM.writeFloat(24, v_i_2); }
void do_tv(char* cmd) { comm.scalar(&target_velocity, cmd); }
void do_K31(char* cmd) { comm.scalar(&LQR_K3_1, cmd); }
void do_K32(char* cmd) { comm.scalar(&LQR_K3_2, cmd); }
void do_K33(char* cmd) { comm.scalar(&LQR_K3_3, cmd); }
void do_K41(char* cmd) { comm.scalar(&LQR_K4_1, cmd); }
void do_K42(char* cmd) { comm.scalar(&LQR_K4_2, cmd); }
void do_K43(char* cmd) { comm.scalar(&LQR_K4_3, cmd); }

void onPacketCallBack(AsyncUDPPacket packet)
{
  char* da;
  da= (char*)(packet.data());
  Serial.println(da);
  comm.run(da);
  EEPROM.commit();
//  packet.print("reply data");
}
// instantiate the commander
void setup() {
   Serial.begin(115200);
   
   //状态灯
   pinMode(ACTIVE_PIN, OUTPUT);
  digitalWrite(ACTIVE_PIN, LOW);
  
  uint32_t chipId = 0;
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  Serial.printf("Chip ID: %d\r\n", chipId);
  
  Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(ESP.getEfuseMac()>>32));//print High 2 bytes
  Serial.printf("%08X\r\n",(uint32_t)ESP.getEfuseMac());//print Low 4bytes. 

  Serial.printf("Chip model = %s Rev %d\r\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores CpuFreqMHz = %u\r\n", ESP.getChipCores(),ESP.getCpuFreqMHz());
  Serial.printf("get Cycle Count = %u\r\n",ESP.getCycleCount());
  Serial.printf("SDK version:%s\r\n", ESP.getSdkVersion());  //获取IDF版本
  
  //获取片内内存 Internal RAM
  Serial.printf("Total heap size = %u\t",ESP.getHeapSize());
  Serial.printf("Available heap = %u\r\n",ESP.getFreeHeap());
  Serial.printf("Lowest level of free heap since boot = %u\r\n",ESP.getMinFreeHeap());
  Serial.printf("Largest block of heap that can be allocated at once = %u\r\n",ESP.getMaxAllocHeap());

  //SPI RAM
  Serial.printf("Total Psram size = %u\t",ESP.getPsramSize());
  Serial.printf("Available Psram = %u\r\n",ESP.getFreePsram());
  Serial.printf("Lowest level of free Psram since boot = %u\r\n",ESP.getMinFreePsram());
  Serial.printf("Largest block of Psram that can be allocated at once = %u\r\n",ESP.getMinFreePsram());
  sprintf(mac_tmp, "%02X\r\n", (uint32_t)(ESP.getEfuseMac() >> (24) ));
  sprintf(mac_tmp, "ESP32-%c%c%c%c%c%c", mac_tmp[4], mac_tmp[5], mac_tmp[2], mac_tmp[3], mac_tmp[0], mac_tmp[1] );
  
   if (!EEPROM.begin(1000)) {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
// eeprom 读取
int k,j;
j = 0;
for(k=0;k<=24;k=k+4)
{
  float nan = EEPROM.readFloat(k);
  if(isnan(nan))
  {
      j = 1;
      Serial.println("frist write");
      EEPROM.writeFloat(0, target_angle);       delay(10);EEPROM.commit();
      EEPROM.writeFloat(4, swing_up_voltage);      delay(10);EEPROM.commit();
      EEPROM.writeFloat(8, swing_up_angle);      delay(10);EEPROM.commit();
      EEPROM.writeFloat(12, v_p_1);      delay(10);EEPROM.commit();
      EEPROM.writeFloat(16, v_i_1);      delay(10);EEPROM.commit();
      EEPROM.writeFloat(20, v_p_2);      delay(10);EEPROM.commit();
      EEPROM.writeFloat(24, v_i_2);       delay(10);EEPROM.commit();
      EEPROM.writeUChar(28,brightness); delay(10);EEPROM.commit();
      EEPROM.writeUChar(32,rgb_flag); delay(10);EEPROM.commit();
  }
}
if(j == 0)
{
     target_angle = EEPROM.readFloat(0);
  swing_up_voltage = EEPROM.readFloat(4);
  swing_up_angle = EEPROM.readFloat(8);  
    v_p_1 = EEPROM.readFloat(12);
  v_i_1 = EEPROM.readFloat(16);
  v_p_2 = EEPROM.readFloat(20);
  v_i_2 = EEPROM.readFloat(24);
  brightness = EEPROM.readUChar(28);
  rgb_flag = EEPROM.readUChar(32);
  motor.PID_velocity.P = v_p_1;
  motor.PID_velocity.I = v_i_1;
}
   //命令设置
 comm.add("TA",do_TA);
 comm.add("START",do_START);
 comm.add("MOTOR",do_MOTOR);
 comm.add("SV",do_SV);
 comm.add("SA",do_SA);
 comm.add("TVQ",do_TVQ);
 comm.add("TVV",do_TVV);
 comm.add("VV",do_VV);
 comm.add("VQ",do_VQ);
//速度环参数
  comm.add("VP1",do_vp1);
  comm.add("VI1",do_vi1);
  comm.add("VP2",do_vp2);
  comm.add("VI2",do_vi2);
  comm.add("TV",do_tv);
  comm.add("K31",do_K31);
  comm.add("K32",do_K32);
  comm.add("K33",do_K33);
  comm.add("K41",do_K41);
  comm.add("K42",do_K42);
  comm.add("K43",do_K43);

//RGB
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(brightness); // Set BRIGHTNESS to about 1/5 (max = 255)
  colorWipe_delay(strip.Color(255, 106, 106),50);
  colorWipe_delay(strip.Color(0, 255, 255),50);
  colorWipe_delay(strip.Color(148, 0, 211),50);

  // kalman mpu6050 init
  Wire.begin(19, 18,400000);// Set I2C frequency to 400kHz
  i2cData[0] = 7;    // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false))
    ; // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true))
    ; // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1))
    ;
  if (i2cData[0] != 0x68)
  { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1)
      ;
  }
  delay(100); // Wait for sensor to stabilize
  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6))
    ;
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  double pitch = acc2rotation(accX, accY);
  kalmanZ.setAngle(pitch);
  gyroZangle = pitch;
  timer = micros();
  Serial.println("kalman mpu6050 init");
  
  I2Ctwo.begin(23, 5, 400000);   //SDA,SCL
  sensor.init(&I2Ctwo);

  //连接motor对象与传感器对象
  motor.linkSensor(&sensor);

  //供电电压设置 [V]
  driver.voltage_power_supply = 12;
  driver.init();

  //连接电机和driver对象
  motor.linkDriver(&driver);

  //FOC模型选择
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  //运动控制模式设置
  motor.controller = MotionControlType::torque;
    //速度PI环设置
  motor.PID_velocity.P = v_p_1;
  motor.PID_velocity.I = v_i_1;

  //最大电机限制电机
  motor.voltage_limit = 12;

  //速度低通滤波时间常数
  motor.LPF_velocity.Tf = 0.01;

  //设置最大速度限制
  motor.velocity_limit = 40;

  motor.useMonitoring(Serial);
  
  //初始化电机
  motor.init();

  //初始化 FOC
  motor.initFOC();

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));

   digitalWrite(ACTIVE_PIN, HIGH);
}
char buf[255];
void loop() {
    motor.loopFOC();   //foc循环用来控制电机运动
  if(wifi_on_off)
    {
      ArduinoOTA.handle();
    }
  // 触摸效果以及RGB灯效
  unsigned long currentMillis = millis();  
  if(currentMillis - voltage_last_time >=1000)
  {
    voltage_last_time = currentMillis;
    voltage_detection();
  }
  if(currentMillis - touch_last_time >= 10) {        //  Check for expired time
  touch_last_time = currentMillis;                            //  Run current frame
  touchAttach(0,T2);
  touchAttach(1,T3);
  touchAttach(2,T4);
    int i;
    for(i = 0;i<3;i++)
    {
      if(touch_STATE[i]&&touch_touched[i])
        if(touch_touched[i] == 1)
        {
          single_event(i);
        }
        else
          long_event(i);
    }
  }
  //  Update current time  更新RGB效果
  if(currentMillis - pixelPrevious >= pixelInterval) {        //  Check for expired time
    pixelPrevious = currentMillis;                            //  Run current frame
    switch(rgb_flag){
    case 0 :
      rgb_off();
      break;
    case 1 :
      if(motor.shaft_velocity>0)
      {
        pixelInterval = 150 - motor.shaft_velocity;
        strip2();
      }
      else
      {
        pixelInterval = 150 + motor.shaft_velocity;
        strip3();  
      }
      break;
    case 2 :
      pixelInterval = 100;
      strip2();
      break;
    case 3 :
      pixelInterval = 100;
      strip3();  
      break;
    case 4 :
      strip1();
      break;
    case 5 :
      rainbow1();  
      break;
    case 6 :
      rainbow2();  
      break;
    case 7 :
      pulse_rainbow1();  
      break;
  }
  }
  
  // 读取MPU6050数据
  while (i2cRead(0x3B, i2cData, 14));
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
//    tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);
    
    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
    
    double pitch = acc2rotation(accX, accY);
    double gyroZrate = gyroZ / 131.0; // Convert to deg/s
    
    kalAngleZ = kalmanZ.getAngle(pitch, gyroZrate + gyroZ_OFF, dt);
    gyroZangle += (gyroZrate + gyroZ_OFF) * dt;
    compAngleZ = 0.93 * (compAngleZ + (gyroZrate + gyroZ_OFF) * dt) + 0.07 * pitch;

    // Reset the gyro angle when it has drifted too much
    if (gyroZangle < -180 || gyroZangle > 180)
      gyroZangle = kalAngleZ;
      
  float pendulum_angle = constrainAngle(fmod(kalAngleZ,120)-target_angle);
  
//   pendulum_angle当前角度与期望角度差值，在差值大的时候进行摇摆，差值小的时候LQR控制电机保持平衡
if(test_flag == 0)//正常控制
{
  if (abs(pendulum_angle) < swing_up_angle) // if angle small enough stabilize 0.5~30°,1.5~90°
  {
     target_velocity = controllerLQR(pendulum_angle, gyroZrate, motor.shaft_velocity);
    if (abs(target_velocity) > 120)
        target_velocity = _sign(target_velocity) * 120;
        
        motor.controller = MotionControlType::velocity;
        motor.move(target_velocity);
  }
  else // else do swing-up
  {    // sets swing_up_voltage to the motor in order to swing up
        motor.controller = MotionControlType::torque;
          target_voltage = -_sign(gyroZrate) * swing_up_voltage;
          motor.move(target_voltage);
  }
}
else if(test_flag == 1)
{

  motor.move(target_voltage);
}
else
{

  motor.move(target_velocity);
}

//串口输出数据部分，不需要的情况可以改为0
#if 0

Serial.print(pitch);Serial.print("\t");
Serial.print(kalAngleZ);Serial.print("\t");
  Serial.print(target_voltage);Serial.print("\t");
  Serial.print(motor.shaft_velocity);Serial.print("\t");
   Serial.print(motor.voltage.q);Serial.print("\t");
  Serial.print(target_angle);Serial.print("\t");
  Serial.print(pendulum_angle);Serial.print("\t");
  Serial.print(gyroZrate);Serial.print("\t");
  Serial.print("\r\n");
#endif
  //可以使用该方法wifi发送udp信息
if(wifi_flag)
{
  memset(buf, 0, strlen(buf));  
 
  wifi_print("v", motor.shaft_velocity);
  wifi_print("vq",motor.voltage.q);
  wifi_print("p",pendulum_angle);
  wifi_print("t",target_angle);
  wifi_print("k",kalAngleZ);
  wifi_print("g",gyroZrate);
  wifi_print("VT",bat_voltage);
  
  udp.writeTo((const unsigned char*)buf, strlen(buf), IPAddress(192,168,4,2), localUdpPort); //广播数据
}
}
/* mpu6050加速度转换为角度
            acc2rotation(ax, ay)
            acc2rotation(az, ay) */
double acc2rotation(double x, double y)
{
  double tmp_kalAngleZ = (atan(x / y) / 1.570796 * 90);
  if (y < 0)
  {
    return (tmp_kalAngleZ + 180);
  }
  else if (x < 0)
  {
    //将当前值与前值比较，当前差值大于100则认为异常
    if (!isnan(kalAngleZ) && (tmp_kalAngleZ + 360 - kalAngleZ) > 100) {
      //Serial.print("X<0"); Serial.print("\t");
      //Serial.print(tmp_kalAngleZ); Serial.print("\t");
      //Serial.print(kalAngleZ); Serial.print("\t");
      //Serial.print("\r\n");
      if (tmp_kalAngleZ < 0 && kalAngleZ < 0) //按键右边角
        return tmp_kalAngleZ;
      else  //按键边异常处理
        return tmp_kalAngleZ;
    } else
      return (tmp_kalAngleZ + 360);
  }
  else
  {
    return tmp_kalAngleZ;
  }
}

// function constraining the angle in between -60~60
float constrainAngle(float x)
{
  float a = 0;
  if(x < 0)
  {
      a = 120+x;
    if(a<abs(x))
      return a;
  }
  return x;
}
// LQR stabilization controller functions
// calculating the voltage that needs to be set to the motor in order to stabilize the pendulum
float controllerLQR(float p_angle, float p_vel, float m_vel)
{
  if (abs(p_angle) > 5) //摆角大于5则进入非稳态，记录非稳态时间
  {
    last_unstable_time = millis();
    if (stable) //如果是稳态进入非稳态则调整为目标角度
    {
      //target_angle = EEPROM.readFloat(0) - p_angle;
      target_angle = EEPROM.readFloat(0);
      stable = 0;
    }
  }
  if ((millis() - last_unstable_time) > 1000 && !stable)  //非稳态进入稳态超过500ms检测，更新目标角为目标角+摆角，假设进入稳态
  {
    //target_angle  -= _sign(target_velocity) * 0.4;
    target_angle = target_angle+p_angle;
    stable = 1;
  }

  if ((millis() - last_stable_time) > 2500 && stable) { //稳态超过2000ms检测，更新目标角
    if (abs(target_velocity) > 5 ) { //稳态速度偏大校正
      last_stable_time = millis();
      target_angle  -= _sign(target_velocity) * 0.2;
    }
  }

  //Serial.println(stable);
  float u;

  if (!stable)  //非稳态计算
  {
    motor.PID_velocity.P = v_p_1;
    motor.PID_velocity.I = v_i_1;
    u = LQR_K3_1 * p_angle + LQR_K3_2 * p_vel + LQR_K3_3 * m_vel;
  }
  else
  {
    motor.PID_velocity.P = v_p_2;
    motor.PID_velocity.I = v_i_2;
    u = LQR_K4_1 * p_angle + LQR_K4_2 * p_vel + LQR_K4_3 * m_vel;
  }

  return u;
}
void wifi_print(char * s,double num)
{
  char str[255];
  char n[255];
  sprintf(n, "%.2f",num);
  strcpy(str,s);
  strcat(str, n);
  strcat(buf+strlen(buf), str);
  strcat(buf, ",\0");

}
void voltage_detection()
{
  #if defined(BAT_VOLTAGE_SENSE_PIN)  //电池电压检测
  bat_voltage = return_voltage_value(BAT_VOLTAGE_SENSE_PIN);
  //driver.voltage_power_supply = bat_voltage;
  //Serial.println(driver.voltage_power_supply);
  if (bat_voltage < min_voltage && !battery_low)
  {
    battery_low = 1;
    Serial.print(driver.voltage_power_supply);
    Serial.println("V ");
    Serial.print(bat_voltage);
    Serial.println("V battery_low!!");
    while (battery_low)
    {
      rgb_off();
      motor.disable();

      bat_voltage = return_voltage_value(BAT_VOLTAGE_SENSE_PIN);
      if (bat_voltage >= (min_voltage + 0.5)) {
        Serial.print(driver.voltage_power_supply);
        Serial.println("V");
        Serial.print(bat_voltage);
        Serial.println("V battery ok");
        digitalWrite(ACTIVE_PIN, 0);  //电池电压恢复则常亮，需reset重启
        //battery_low = 0;
      } else {  //电池电压低闪灯
        if (millis() % 500 < 250)
          digitalWrite(ACTIVE_PIN, 0);
        else
          digitalWrite(ACTIVE_PIN, 1);
      }
    }
  }
#endif
}
double return_voltage_value(int pin_no)
{
  double tmp;
  double ADCVoltage;
  double inputVoltage;
  analogSetPinAttenuation(pin_no, ADC_6db);

  for (int i = 0; i < 20; i++)
  {
    ADCVoltage = analogReadMilliVolts(pin_no) / 1000.0;
    inputVoltage = (ADCVoltage * R1_VOLTAGE) / R2_VOLTAGE;

    tmp = tmp + inputVoltage + ADCVoltage; // formula for calculating voltage in i.e. GND
  }
  inputVoltage = tmp / 20;
  if(inputVoltage!=0)
    inputVoltage = inputVoltage + 0.001;
/*

  for (int i = 0; i < 20; i++)
  {
    tmp = tmp + analogRead(pin_no);
  }
  tmp = tmp / 20;

  ADCVoltage = ((tmp * 3.3) / 4095.0) + 0.165;
  inputVoltage = ADCVoltage / (R2_VOLTAGE / (R1_VOLTAGE + R2_VOLTAGE)); // formula for calculating voltage in i.e. GND
*/

  return inputVoltage;
}
void AutoWifiConfig()
{
   //wifi初始化
   sprintf(mac_tmp, "%02X\r\n", (uint32_t)(ESP.getEfuseMac() >> (24) ));
  sprintf(mac_tmp, "ESP32-%c%c%c%c%c%c", mac_tmp[4], mac_tmp[5], mac_tmp[2], mac_tmp[3], mac_tmp[0], mac_tmp[1] );

  WiFi.mode(WIFI_AP);
  while (!WiFi.softAP(ssid, password)) {}; //启动AP
  Serial.println("AP启动成功");
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
  byte mac[6];
  WiFi.macAddress(mac);
  WiFi.setHostname(ServerName);
  Serial.printf("macAddress 0x%02X:0x%02X:0x%02X:0x%02X:0x%02X:0x%02X\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  
  while (!udp.listen(localUdpPort)) //等待udp监听设置成功
  {
  }
  udp.onPacket(onPacketCallBack); //注册收到数据包事件

  ArduinoOTA.setHostname(ServerName);
  //以下是启动OTA，可以通过WiFi刷新固件
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}
//触摸单击函数处理
void single_event(int touchID)
{
  switch(touchID){
    case 0  :
       if(brightness<=15)
        brightness = 15;
       brightness-=15;
       EEPROM.writeUChar(28, brightness);  EEPROM.commit();
       strip.setBrightness(brightness); // Set BRIGHTNESS to about 1/5 (max = 255)
       break; 
    case 1  :
      if(brightness>=240)
            brightness = 240;
       brightness+=15;      
       EEPROM.writeUChar(28, brightness);  EEPROM.commit();
       strip.setBrightness(brightness); // Set BRIGHTNESS to about 1/5 (max = 255)
       break; 
    case 2 :
       if(rgb_flag)
          rgb_flag = 0;
       else
          rgb_flag = EEPROM.readUChar(32);
       break; 
  }  
}
//触摸长按函数处理
void long_event(int touchID)
{
  switch(touchID){
    case 0  : //长按投币
       if(rgb_flag == 0)
          rgb_flag = rgb_modle;
       rgb_flag--;
       strip.setBrightness(brightness); // Set BRIGHTNESS to about 1/5 (max = 255)
       EEPROM.writeUChar(32, rgb_flag);  EEPROM.commit();
       break; 
    case 1  :  //长按收藏
       rgb_flag++;
       if(rgb_flag>=rgb_modle)
          rgb_flag = 0;
       strip.setBrightness(brightness); // Set BRIGHTNESS to about 1/5 (max = 255)
       EEPROM.writeUChar(32, rgb_flag);  EEPROM.commit();
       break; 
    case 2 : //长按点赞
       if(wifi_on_off)
        {
           motor.enable();
          WiFi.disconnect();
          WiFi.mode(WIFI_OFF);
          Serial.println("WIFI_OFF");
        }
       else
       {
        motor.disable();
        AutoWifiConfig();//打开wifi
        Serial.println("WIFI_ON");
       }
       wifi_on_off = !wifi_on_off;
       Motor_enable_flag = !Motor_enable_flag;
       break; 
  }  
}
