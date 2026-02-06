# 使用 MotionCal 校准磁场数据指南

* [English Version](./README.md)

磁力计在现实应用中易受金属物体、电流干扰和地磁场波动的影响。未经校准的数据可能会导致方向识别偏移，影响航向角（yaw）或电子罗盘功能的准确性。

---

## 所需工具

* [MotionCal](https://www.pjrc.com/store/prop_shield.html#motioncal) 工具（支持 Windows/macOS/Linux）
* Arduino IDE
* DFRobot_BMM350传感器器所支持的开发板，并将串口正确连接到PC

---

## 步骤一：上传校准固件

DFRobot_BMM350传感器器所支持的开发板，并将串口正确连接到PC

```cpp
#include "DFRobot_BMM350.h"

DFRobot_BMM350_I2C bmm350(&Wire, I2C_ADDRESS);
void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  while (bmm350.begin()) {
    Serial.println("bmm350 init failed, Please try again!");
    delay(1000);
  }
  Serial.println("bmm350 init success!");

  /**
   * Set sensor operation mode
   * opMode:
   *   eBmm350SuspendMode      // suspend mode: Suspend mode is the default power mode of BMM350 after the chip is powered, Current consumption in suspend mode is minimal, 
   *                               so, this mode is useful for periods when data conversion is not needed. Read and write of all registers is possible.
   *   eBmm350NormalMode       // normal mode  Get geomagnetic data normally.
   *   eBmm350ForcedMode       // forced mode  Single measurement, the sensor restores to suspend mode when the measurement is done.
   *   eBmm350ForcedModeFast  // To reach ODR = 200Hz is only possible by using FM_ FAST.
   */
  bmm350.setOperationMode(eBmm350NormalMode);

  /**
   * Set preset mode, make it easier for users to configure sensor to get geomagnetic data (The default rate for obtaining geomagnetic data is 12.5Hz)
   * presetMode:
   *   BMM350_PRESETMODE_LOWPOWER      // Low power mode, get a fraction of data and take the mean value.
   *   BMM350_PRESETMODE_REGULAR       // Regular mode, get a number of data and take the mean value.
   *   BMM350_PRESETMODE_ENHANCED      // Enhanced mode, get a plenty of data and take the mean value.
   *   BMM350_PRESETMODE_HIGHACCURACY  // High accuracy mode, get a huge number of take and draw the mean value.
   */
  bmm350.setPresetMode(BMM350_PRESETMODE_HIGHACCURACY,BMM350_DATA_RATE_25HZ);

  /**
   * Enable the measurement at x-axis, y-axis and z-axis, default to be enabled, no config required, the geomagnetic data at x, y and z will be inaccurate when disabled.
   * Refer to setMeasurementXYZ() function in the .h file if you want to configure more parameters.
   */
  bmm350.setMeasurementXYZ();
}

void loop() {
  sBmm350MagData_t magData = bmm350.getGeomagneticData();
    Serial.print("Raw:");
    Serial.print(0);
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');
    Serial.print(magData.x*10);
    Serial.print(',');
    Serial.print(magData.y*10);
    Serial.print(',');
    Serial.print(magData.z*10);
    Serial.println();
  delay(100);
}
```

---

## 步骤二：运行 MotionCal 工具

1. 打开 [MotionCal](https://www.pjrc.com/store/prop_shield.html#motioncal) 下载页面，选择你的平台并下载安装。

![MotionCal download pic](/resources/images/cal_pic1.jpg)

2. 启动 `MotionCal`。
3. 在菜单中选择正确的串口（与你的设备一致）。

![MotionCal port choose](/resources/images/cal_pic2.jpg)

4. MotionCal 自动开始自动接收并可视化磁力计、加速度计和陀螺仪数据。

> 确保没有其它串口软件同时打开!!

---

## 步骤三：旋转传感器进行全向采样

> 将传感器沿 **X/Y/Z 各轴方向**缓慢旋转，使数据图形覆盖完整的球体。

![MotionCal cal](/resources/images/cal_pic3.jpg)

## 步骤四：在代码中应用补偿

![calibration parameters](/resources/images/cal_pic4.jpg)

MotionCal软件左上角即为所需的校正参数
分别将校正系数填入参考代码`CalibrateMagnedticData.ino`,对应位置

```cpp
//hard iron calibration parameters
const float hard_iron[3] = { -13.45, -28.95, 12.69 };
//soft iron calibration parameters
const float soft_iron[3][3] = {
  { 0.992, -0.006, -0.007 },
  { -0.006, 0.990, -0.004 },
  { -0.007, -0.004, 1.019 }
};
```

完整参考代码如下：

```cpp
#include "DFRobot_BMM350.h"

DFRobot_BMM350_I2C bmm350(&Wire, I2C_ADDRESS);
//hard iron calibration parameters
const float hard_iron[3] = { -13.45, -28.95, 12.69 };
//soft iron calibration parameters
const float soft_iron[3][3] = {
  { 0.992, -0.006, -0.007 },
  { -0.006, 0.990, -0.004 },
  { -0.007, -0.004, 1.019 }
};

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  while (bmm350.begin()) {
    Serial.println("bmm350 init failed, Please try again!");
    delay(1000);
  }
  Serial.println("bmm350 init success!");

  /**
   * Set sensor operation mode
   * opMode:
   *   eBmm350SuspendMode      // suspend mode: Suspend mode is the default power mode of BMM350 after the chip is powered, Current consumption in suspend mode is minimal, 
   *                               so, this mode is useful for periods when data conversion is not needed. Read and write of all registers is possible.
   *   eBmm350NormalMode       // normal mode  Get geomagnetic data normally.
   *   eBmm350ForcedMode       // forced mode  Single measurement, the sensor restores to suspend mode when the measurement is done.
   *   eBmm350ForcedModeFast  // To reach ODR = 200Hz is only possible by using FM_ FAST.
   */
  bmm350.setOperationMode(eBmm350NormalMode);

  /**
   * Set preset mode, make it easier for users to configure sensor to get geomagnetic data (The default rate for obtaining geomagnetic data is 12.5Hz)
   * presetMode:
   *   BMM350_PRESETMODE_LOWPOWER      // Low power mode, get a fraction of data and take the mean value.
   *   BMM350_PRESETMODE_REGULAR       // Regular mode, get a number of data and take the mean value.
   *   BMM350_PRESETMODE_ENHANCED      // Enhanced mode, get a plenty of data and take the mean value.
   *   BMM350_PRESETMODE_HIGHACCURACY  // High accuracy mode, get a huge number of take and draw the mean value.
   */
  bmm350.setPresetMode(BMM350_PRESETMODE_HIGHACCURACY,BMM350_DATA_RATE_25HZ);

  /**
   * Enable the measurement at x-axis, y-axis and z-axis, default to be enabled, no config required, the geomagnetic data at x, y and z will be inaccurate when disabled.
   * Refer to setMeasurementXYZ() function in the .h file if you want to configure more parameters.
   */
  bmm350.setMeasurementXYZ();
}

void loop() {
  sBmm350MagData_t magData = bmm350.getGeomagneticData();

  float mag_data[3];

  // hard iron calibration
  mag_data[0] = magData.float_x + hard_iron[0];
  mag_data[1] = magData.float_y + hard_iron[1];
  mag_data[2] = magData.float_z + hard_iron[2];

  //soft iron calibration
  for (int i = 0; i < 3; i++) {
    mag_data[i] = (soft_iron[i][0] * mag_data[0]) + (soft_iron[i][1] * mag_data[1]) + (soft_iron[i][2] * mag_data[2]);
  }

  magData.x = mag_data[0];
  magData.y = mag_data[1];
  magData.z = mag_data[2];
  magData.float_x = mag_data[0];
  magData.float_y = mag_data[1];
  magData.float_z = mag_data[2];

  Serial.print("mag x = ");Serial.print(magData.x);Serial.println(" uT");
  Serial.print("mag y = ");Serial.print(magData.y);Serial.println(" uT");
  Serial.print("mag z = ");Serial.print(magData.z);Serial.println(" uT");

  // float type data
  //Serial.print("mag x = "); Serial.print(magData.float_x); Serial.println(" uT");
  //Serial.print("mag y = "); Serial.print(magData.float_y); Serial.println(" uT");
  //Serial.print("mag z = "); Serial.print(magData.float_z); Serial.println(" uT");

  float compassDegree = getCompassDegree(magData);
  Serial.print("the angle between the pointing direction and north (counterclockwise) is:");
  Serial.println(compassDegree);
  Serial.println("--------------------------------");
  delay(3000);
}
float getCompassDegree(sBmm350MagData_t magData)
{
    float compass = 0.0;
    compass = atan2(magData.x, magData.y);
    if (compass < 0) {
        compass += 2 * PI;
    }
    if (compass > 2 * PI) {
        compass -= 2 * PI;
    }
    return compass * 180 / M_PI;
}
```

---

---

## 📎 附录

* MotionCal 下载地址：[https://www.pjrc.com/store/prop\_shield.html#motioncal](https://www.pjrc.com/store/prop_shield.html#motioncal)
* DFRobot BMM350 Sensor：[https://wiki.dfrobot.com.cn/_SKU_SEN0619_Gravity_BMM350_TripleAxis_Magnetometer_Sensor](https://wiki.dfrobot.com.cn/_SKU_SEN0619_Gravity_BMM350_TripleAxis_Magnetometer_Sensor)
