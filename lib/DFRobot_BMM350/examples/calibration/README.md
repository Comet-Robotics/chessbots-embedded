# Guide to Calibrating BMM350 Magnetic Field Data Using MotionCal

* [中文版本](./README_CN.md)

Magnetometers in real-world applications are susceptible to interference from metal objects, electrical currents, and fluctuations in the Earth's magnetic field. Uncalibrated data can lead to deviations in direction recognition, affecting the accuracy of the heading angle (yaw) or the functionality of the electronic compass.

---

## Required Tools

* [MotionCal](https://www.pjrc.com/store/prop_shield.html#motioncal) tool (supports Windows/macOS/Linux)
* Arduino IDE
* A development board supported by the DFRobot_BMM350 sensor, with the serial port correctly connected to the PC

---

## Step 1: Upload the Calibration Firmware

Use a development board supported by the DFRobot_BMM350 sensor and connect the serial port to the PC correctly.

---

## Step 2: Run the MotionCal Tool

1. Open the [MotionCal](https://www.pjrc.com/store/prop_shield.html#motioncal) download page, select your platform, and download and install the tool.

![MotionCal download pic](/resources/images/cal_pic1.jpg)

2. Launch `MotionCal`.
3. Select the correct serial port in the menu (consistent with your device).

![MotionCal port choose](/resources/images/cal_pic2.jpg)

4. MotionCal automatically starts receiving and visualizing the data from the magnetometer, accelerometer, and gyroscope.

> Ensure that no other serial port software is open simultaneously!

---

## Step 3: Rotate the Sensor for Omnidirectional Sampling

> Slowly rotate the sensor along the **X/Y/Z axes** to ensure that the data graph covers a complete sphere.

![MotionCal cal](/resources/images/cal_pic3.jpg)

## Step 4: Apply Compensation in the Code

![calibration parameters](/resources/images/cal_pic4.jpg)

The calibration parameters required are located in the top-left corner of the MotionCal software.
Fill in the calibration coefficients in the corresponding positions of the reference code `CalibrateMagnedticData.ino`.

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

The complete reference code is as follows:

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

## 📎 Appendix

* MotionCal download link: [https://www.pjrc.com/store/prop_shield.html#motioncal](https://www.pjrc.com/store/prop_shield.html#motioncal)
* DFRobot BMM350 Sensor: [https://wiki.dfrobot.com.cn/_SKU_SEN0619_Gravity_BMM350_TripleAxis_Magnetometer_Sensor](https://wiki.dfrobot.com.cn/_SKU_SEN0619_Gravity_BMM350_TripleAxis_Magnetometer_Sensor)