# DFRobot_bmm350

- [English Version](./README.md)

BMM350 是一款低功耗、低噪声的 3 轴数字地磁传感器，完全符合罗盘应用的要求。 基于博世专有的 FlipCore 技术，BMM350 提供了高精度和动态的绝对空间方向和运动矢量。 体积小、重量轻，特别适用于支持无人机精准航向。 BMM350 还可与由 3 轴加速度计和 3 轴陀螺仪组成的惯性测量单元一起使用。

![产品效果图](../../resources/images)![产品效果图](../../resources/images)


## 产品链接（[https://www.dfrobot.com.cn](https://www.dfrobot.com.cn)）
    SKU: 

## 目录

  * [概述](#概述)
  * [库安装](#库安装)
  * [方法](#方法)
  * [兼容性](#兼容性)
  * [历史](#历史)
  * [创作者](#创作者)

## 概述

您可以沿 XYZ 轴获取地磁数据

1. 本模块可以获得高阈值和低阈值地磁数据。 <br>
2. 可以测量三个（xyz）轴上的地磁。<br>
3. 本模块可选择I2C或I3C通讯方式。<br> 


## 库安装
1. 下载库至树莓派，要使用这个库，首先要将库下载到Raspberry Pi，命令下载方法如下:<br>
```python
sudo git clone https://github.com/DFRobot/DFRobot_BMM350
```
2. 打开并运行例程，要执行一个例程demo_x.py，请在命令行中输入python demo_x.py。例如，要执行data_ready_interrupt.py例程，你需要输入:<br>

```python
python3 data_ready_interrupt.py 
```

## 方法

```python
  '''!
    @brief 初始化bmm350 判断芯片id是否正确
    @return 0  is init success
    @n      -1 is init failed
  '''
  def sensor_init(self):

  '''!
    @brief 软件复位，软件复位后先恢复为挂起模式（需手动进行普通模式）
  '''
  def soft_reset(self):

  '''!
    @brief 传感器自测，返回字符串表明自检结果
    @return 测试结果的字符串
  '''
  def self_test(self):

  '''!
    @brief 设置传感器的执行模式
    @param modes
    @n     BMM350_SUSPEND_MODE      挂起模式: 挂起模式是芯片上电后BMM350的默认电源模式，在挂起模式下电流消耗最小，因此，这种模式在不需要进行数据转换时非常有用。所有寄存器的读写是可能的
    @n     BMM350_NORMAL_MODE       普通模式: 正常获取地磁数据
    @n     BMM350_FORCED_MODE       强制模式: 单次测量，测量完成后传感器恢复到暂停模式
    @n     BMM350_FORCED_MODE_FAST  只有使用FM_ FAST才能达到ODR = 200Hz
  '''
  def set_operation_mode(self, modes):

  '''!
    @brief 获取传感器的执行模式
    @return 返回模式的字符串
  '''
  def get_operation_mode(self):

  '''!
    @brief 设置地磁数据获取的速率，速率越大获取越快(不加延时函数)
    @param rate
    @n BMM350_DATA_RATE_1_5625HZ
    @n BMM350_DATA_RATE_3_125HZ
    @n BMM350_DATA_RATE_6_25HZ
    @n BMM350_DATA_RATE_12_5HZ  (default rate)
    @n BMM350_DATA_RATE_25HZ
    @n BMM350_DATA_RATE_50HZ
    @n BMM350_DATA_RATE_100HZ
    @n BMM350_DATA_RATE_200HZ
    @n BMM350_DATA_RATE_400HZ
  '''
  def set_rate(self, rates):

  '''!
    @brief 获取配置的数据速率 单位：HZ
    @return rate
  '''
  def get_rate(self):

  '''!
    @brief 设置预置模式，使用户更简单的配置传感器来获取地磁数据 (默认的采集速率为12.5Hz)
    @param modes 
    @n     BMM350_PRESETMODE_LOWPOWER       低功率模式,获取少量的数据 取均值
    @n     BMM350_PRESETMODE_REGULAR        普通模式,获取中量数据 取均值
    @n     BMM350_PRESETMODE_ENHANCED       增强模式,获取大量数据 取均值
    @n     BMM350_PRESETMODE_HIGHACCURACY   高精度模式,获取超大量数据 取均值
  '''
  def set_preset_mode(self, modes):

  '''!
    @brief 使能x y z 轴的测量，默认设置为使能不需要配置，禁止后xyz轴的地磁数据不准确
    @param en_x
    @n     BMM350_X_EN     使能 x 轴的测量
    @n     BMM350_X_DIS    禁止 x 轴的测量
    @param channel_y
    @n     BMM350_Y_EN     使能 y 轴的测量
    @n     BMM350_Y_DIS    禁止 y 轴的测量
    @param channel_z
    @n     BMM350_Z_EN     使能 z 轴的测量
    @n     BMM350_Z_DIS    禁止 z 轴的测量
  '''
  def set_measurement_XYZ(self, en_x = BMM350_X_EN, en_y = BMM350_Y_EN, en_z = BMM350_Z_EN):

  '''!
    @brief 获取 x y z 轴的使能状态
    @return 返回xyz 轴的使能状态的字符串
  '''
  def get_measurement_state_XYZ(self):

  '''!
    @brief 获取x y z 三轴的地磁数据
    @return x y z 三轴的地磁数据的列表 单位：微特斯拉（uT）
    @n      [0] x 轴地磁的数据
    @n      [1] y 轴地磁的数据
    @n      [2] z 轴地磁的数据
  '''
  def get_geomagnetic_data(self):

  '''!
    @brief 获取罗盘方向
    @return 罗盘方向 (0° - 360°)  0° = North, 90° = East, 180° = South, 270° = West.
  '''
  def get_compass_degree(self):

  '''!
    @brief 使能或者禁止数据准备中断引脚
    @n     使能后有数据来临DRDY引脚跳变
    @n     禁止后有数据来临DRDY不进行跳变
    @n     高极性：高电平为活动电平，默认为低电平，触发中断时电平变为高
    @n     低极性：低电平为活动电平，默认为高电平，触发中断时电平变为低
    @param modes
    @n     BMM350_ENABLE_INTERRUPT      使能DRDY
    @n     BMM350_DISABLE_INTERRUPT     禁止DRDY
    @param polarity
    @n     BMM350_ACTIVE_HIGH    高极性
    @n     BMM350_ACTIVE_LOW     低极性
  '''
  def set_data_ready_pin(self, modes, polarity):

  '''!
    @brief 获取数据准备的状态，用来判断数据是否准备好
    @return status
    @n True 表示数据已准备好
    @n False 表示数据还未准备好
  '''
  def get_data_ready_state(self):

  '''!
    @brief 设置阈值中断，当某个通道的地磁值高/低于阈值时触发中断
    @n     高极性：高电平为活动电平，默认为低电平，触发中断时电平变为高
    @n     低极性：低电平为活动电平，默认为高电平，触发中断时电平变为低
    @param modes
    @n     LOW_THRESHOLD_INTERRUPT     低阈值中断模式
    @n     HIGH_THRESHOLD_INTERRUPT    高阈值中断模式
    @param threshold  阈值，默认扩大16倍，例如：低阈值模式下传入阈值1，实际低于16的地磁数据都会触发中断
    @param polarity
    @n     POLARITY_HIGH               高极性
    @n     POLARITY_LOW                低极性
  '''
  def set_threshold_interrupt(self, modes, threshold, polarity):

  '''!
    @brief 获取发生阈值中断的数据
    @return 返回存放地磁数据的列表，列表三轴当数据和中断状态，
    @n      [0] x 轴触发阈值的数据 ，当数据为NO_DATA时为触发中断
    @n      [1] y 轴触发阈值的数据 ，当数据为NO_DATA时为触发中断
    @n      [2] z 轴触发阈值的数据 ，当数据为NO_DATA时为触发中断
  '''
  def get_threshold_data(self):
```

## 兼容性

| 主板         | 通过 | 未通过 | 未测试 | 备注 |
| ------------ | :--: | :----: | :----: | :--: |
| RaspberryPi2 |      |        |   √    |      |
| RaspberryPi3 |      |        |   √    |      |
| RaspberryPi4 |  √   |        |        |      |

* Python 版本

| Python  | 通过 | 未通过 | 未测试 | 备注 |
| ------- | :--: | :----: | :----: | ---- |
| Python3 |  √   |        |        |      |

## 历史

- 2024/05/11 - 1.0.0 版本

## 创作者

Written by [GDuang](yonglei.ren@dfrobot.com), 2024. (Welcome to our [website](https://www.dfrobot.com/))






