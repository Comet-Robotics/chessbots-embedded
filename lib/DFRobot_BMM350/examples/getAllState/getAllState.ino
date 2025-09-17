 /*!
  * @file  getAllState.ino
  * @brief Get all the config status, self test status; the sensor turns to sleep mode from normal mode after reset
  * @n Experimental phenomenon: serial print the sensor config information and the self-test information
  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @license     The MIT License (MIT)
  * @author      [GDuang](yonglei.ren@dfrobot.com)
  * @version     V1.0.0
  * @date        2024-05-06
  * @url         https://github.com/DFRobot/DFRobot_BMM350
  */
#include "DFRobot_BMM350.h"

DFRobot_BMM350_I2C bmm350(&Wire, I2C_ADDRESS);

void setup() 
{
  Serial.begin(115200);
  while(!Serial);
  while(bmm350.begin()){
    Serial.println("bmm350 init failed, Please try again!");
    delay(1000);
  } Serial.println("bmm350 init success!");

  /**
   * Sensor self test, the returned character string indicates the test result.
   */
  Serial.println(bmm350.selfTest());

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
   * rate:
   *   BMM350_DATA_RATE_1_5625HZ
   *   BMM350_DATA_RATE_3_125HZ
   *   BMM350_DATA_RATE_6_25HZ
   *   BMM350_DATA_RATE_12_5HZ   (default rate)
   *   BMM350_DATA_RATE_25HZ
   *   BMM350_DATA_RATE_50HZ
   *   BMM350_DATA_RATE_100HZ
   *   BMM350_DATA_RATE_200HZ
   *   BMM350_DATA_RATE_400HZ
   */
  bmm350.setPresetMode(BMM350_PRESETMODE_HIGHACCURACY,BMM350_DATA_RATE_25HZ);

  /**
   * Enable the measurement at x-axis, y-axis and z-axis, default to be enabled, no config required, the geomagnetic data at x, y and z will be inaccurate when disabled.
   * Refer to setMeasurementXYZ() function in the .h file if you want to configure more parameters.
   */
  bmm350.setMeasurementXYZ();

  /**
   * Get the config data rate unit: HZ
   */
  float rate = bmm350.getRate();
  Serial.print("rate is "); Serial.print(rate); Serial.println(" HZ");

  /**
   * Get the measurement status at x-axis, y-axis and z-axis, return the measurement status as character string
   */
  Serial.println(bmm350.getMeasurementStateXYZ());
  
  /**
   * Get the sensor operation mode, return the sensor operation status as character string
   */
  Serial.println(bmm350.getOperationMode());

  /**
   * After the software is reset, it enters the suspended mode.
   */
  bmm350.softReset();
}

void loop() 
{
  /**
   * Get the sensor operation mode, return the sensor operation status as character string
   */
  Serial.println(bmm350.getOperationMode());
  delay(3000);
}
