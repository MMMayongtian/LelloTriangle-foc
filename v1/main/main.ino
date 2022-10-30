  /**
arduino开发环境-灯哥开源FOChttps://gitee.com/ream_d/Deng-s-foc-controller，并安装Kalman。
FOC引脚32, 33, 25, 22    22为enable
AS5600霍尔传感器 SDA-23 SCL-5  MPU6050六轴传感器 SDA-19 SCL-18
本程序有两种平衡方式， FLAG_V为1时使用电压控制，为0时候速度控制。电压控制时LQR参数使用K1和K2，速度控制时LQR参数使用K3和K4
在wifi上位机窗口中输入：TA+角度，就可以修改平衡角度
比如让平衡角度为90度，则输入：TA90，并且会存入eeprom的位置0中 注：wifi发送命令不能过快，因为每次都会保存进eeprom
在使用自己的电机时，请一定记得修改默认极对数，即 BLDCMotor(5) 中的值，设置为自己的极对数数字，磁铁数量/2
程序默认设置的供电电压为 12V,用其他电压供电请记得修改 voltage_power_supply , voltage_limit 变量中的值
默认PID针对的电机是 GB2204 ，使用自己的电机需要修改PID参数，才能实现更好效果
 */
#include <SimpleFOC.h>
#include "Command.h"
#include <WiFi.h>
#include <AsyncUDP.h> //引用以使用异步UDP
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include "EEPROM.h"
Kalman kalmanZ;
#define gyroZ_OFF -0.19
/* ----IMU Data---- */

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
bool stable = 0;
uint32_t last_unstable_time;
uint32_t last_stable_time;
double gyroZangle; // Angle calculate using the gyro only
double compAngleZ; // Calculated angle using a complementary filter
double kalAngleZ;  // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
/* ----FOC Data---- */

// driver instance
double acc2rotation(double x, double y);
float constrainAngle(float x);
const char *ssid = "esp32";
const char *password = "12345678";

bool wifi_flag = 0;
AsyncUDP udp;                     //创建UDP对象
unsigned int localUdpPort = 2333; //本地端口号
void wifi_print(char * s,double num);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Ctwo = TwoWire(1);
LowPassFilter lpf_throttle{0.00};

//倒立摆参数
float LQR_K3_1 = 10;   //摇摆到平衡
float LQR_K3_2 = 1.7;   //
float LQR_K3_3 = 1.75; //

float LQR_K4_1 = 2.4;   //摇摆到平衡
float LQR_K4_2 = 1.5;   //
float LQR_K4_3 = 1.42; //

//电机参数
BLDCMotor motor = BLDCMotor(5);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);
float target_velocity = 0;
float target_angle = 89.3;
float target_voltage = 0;
float swing_up_voltage = 1.8;
float swing_up_angle = 20;
float v_i_1 = 20;
float v_p_1 = 0.5;
float v_i_2 = 10;
float v_p_2 = 0.2;
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
    test_flag = 1;
}
void do_TVV(char* cmd)
{
  if(test_flag == 2)
    test_flag = 0;
  else
    test_flag = 2;
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

    //wifi初始化
  WiFi.mode(WIFI_AP);
   while(!WiFi.softAP(ssid, password)){}; //启动AP
    Serial.println("AP启动成功");
  while (!udp.listen(localUdpPort)) //等待udp监听设置成功
  {
  }
  udp.onPacket(onPacketCallBack); //注册收到数据包事件
  
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
  motor.controller = MotionControlType::velocity;
    //速度PI环设置
  motor.PID_velocity.P = v_p_1;
  motor.PID_velocity.I = v_i_1;

  //最大电机限制电机
  motor.voltage_limit = 12;

  //速度低通滤波时间常数
  motor.LPF_velocity.Tf = 0.02;

  //设置最大速度限制
  motor.velocity_limit = 40;

  motor.useMonitoring(Serial);
  
  //初始化电机
  motor.init();

  //初始化 FOC
  motor.initFOC();

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));

}
char buf[255];
long loop_count = 0;
double last_pitch;
void loop() {
    motor.loopFOC();
  if (1)
  {
//    loop_count++ == 10
//    loop_count = 0;
  while (i2cRead(0x3B, i2cData, 14));
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);
    
    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
    
    double pitch = acc2rotation(accX, accY);
    double gyroZrate = gyroZ / 131.0; // Convert to deg/s
    if(abs(pitch-last_pitch)>100)
      kalmanZ.setAngle(pitch);
    
    kalAngleZ = kalmanZ.getAngle(pitch, gyroZrate + gyroZ_OFF, dt);
    last_pitch = pitch;
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
    if (abs(target_velocity) > 140)
        target_velocity = _sign(target_velocity) * 140;
        
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
  motor.controller = MotionControlType::torque;
  motor.move(target_voltage);
}
else
{
  motor.controller = MotionControlType::velocity;
  motor.move(target_velocity);
}
//串口输出数据部分，不需要的情况可以改为0
#if 1

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

  udp.writeTo((const unsigned char*)buf, strlen(buf), IPAddress(192,168,4,2), localUdpPort); //广播数据
  }
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
