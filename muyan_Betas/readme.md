# RGB自平衡莱洛三角形

#### 介绍

本版本是慕炎RGB版本程序 基于45°([https://gitee.com/coll45/foc/](https://gitee.com/coll45/foc/))程序修改

本版本虽然能在原版本pcb上运行，但完整功能需要对应的硬件支持才能获得

 **硬件要点** 

电机连接线线序按原插头顺序安装

动量轮的重量，建议安装所有孔位螺丝及螺母

磁铁的磁性为径向，而不是正反两面为NS

AS5600与磁铁的距离 0.3-3mm之内

可通过TTL观察simplefoc初始化时的状态提示判断，如果为PP Check：fail则需要检查磁铁及安装情况


【硬件基于45°工程文件修改的RGB版本】

[https://oshwhub.com/muyan2020/zi-ping-heng-di-lai-luo-san-jiao_10-10-ban-ben_copy](https://oshwhub.com/muyan2020/zi-ping-heng-di-lai-luo-san-jiao_10-10-ban-ben_copy)


1、去掉ch340和自动下载电路更换为rx tx插针，使用时不要连接3.3v插针，烧录前按住boot不放，再按一下reset，然后放掉所有键（如果打开串口监视发现数据乱码，把rx和tx线对换）

2、TPS54331更改为5V输出供电给RGB灯，5V输入到ams1117-3.3

3、预留2个触摸区和增加esp32状态灯，KEY3触摸开关灯，KEY4长按调光

4、无线充电功能（无线充电底座）


#### 软件架构

1、OTA

2、RGB灯控制程序

3、触摸控制程序

4、基于webserver的基础调参功能


#### webserver安装教程

webserver需要使用【ESP32 SPIFFS】文件上传

ESP32 SPIFFS文件上传方法

下载插件复制到C:\Program Files (x86)\Arduino\tools\ESP32FS\tool

重启Arduino即可

arduino-esp32fs-plugin

https://github.com/me-no-dev/arduino-esp32fs-plugin/releases/tag/1.0

使用时在arduino界面点“工具”-“ESP32 Sketch Data Upload”

#### 使用说明

Key3长按为开关RGB灯光

key4长按为增加亮度，放手后再次触摸为降低亮度

web页面调参

通过浏览器访问:[http://192.168.4.1](http://192.168.4.1)，用户名：admin 密码：reuleaux123(可通过修改代码修改)


#### 更新说明

### 20211229


1、增加OTA升级，第一次烧完固件后，再也不需要串口线了

2、增加webserver，通过手机浏览器访问192.168.4.1，直接可以进行数据查看和调参（调参功能还没做）


### 20220221

增加V2版本，电机是2715，对应极数7

V2版本的第三个按键，设定为wifi开关，按住2秒，ACT灯由亮转暗则设定完成


### 20220223

V1目录改为V1.5 为 GB2204 RGB版本

V1.5 V2修正摇摆无法平衡的问题

V2 默认不开启wifi，平衡后电流在50ma左右，开启wifi在100ma左右


