# DFRobot_bmm350

* [中文](./README_CN.md)

This RaspberryPi BMM350 sensor board can communicate with RaspberryPi via I2C or I3C.<br>
The BMM350 is capable of obtaining triaxial geomagnetic data.<br>

![产品效果图](../../resources/images/)![产品效果图](../../resources/images/)

## Product Link（[https://www.dfrobot.com](https://www.dfrobot.com)）
    SKU：

## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [History](#history)
* [Credits](#credits)

## Summary

Get geomagnetic data along the XYZ axis.

1. This module can obtain high threshold and low threshold geomagnetic data. <br>
2. Geomagnetism on three(xyz) axes can be measured.<br>
3. This module can choose I2C or I3C communication mode.<br>


## Installation

This Sensor should work with DFRobot_BMM350 on RaspberryPi. <br>
Run the program:

```
$> python3 get_all_state.py
$> python3 data_ready_interrupt.py
$> python3 get_geomagnetic_data.py
$> python3 threshold_interrupt.py
```

## Methods

```python
  '''!
    @brief Init bmm350 check whether the chip id is right
    @return 0  is init success
            -1 is init failed
  '''
  def sensor_init(self):

  '''!
    @brief Soft reset, restore to suspended mode after soft reset. (You need to manually enter the normal mode)
  '''
  def soft_reset(self):

  '''!
    @brief Sensor self test, the returned character string indicate the self test result.
    @return The character string of the test result
  '''
  def self_test(self):

  '''!
    @brief Set sensor operation mode
    @param modes
    @n BMM350_SUSPEND_MODE      suspend mode: Suspend mode is the default power mode of BMM350 after the chip is powered, Current consumption in suspend mode is minimal, 
                                so, this mode is useful for periods when data conversion is not needed. Read and write of all registers is possible.
    @n BMM350_NORMAL_MODE       normal mode: Get geomagnetic data normally.      
    @n BMM350_FORCED_MODE       forced mode: Single measurement, the sensor restores to suspend mode when the measurement is done.
    @n BMM350_FORCED_MODE_FAST  To reach ODR = 200Hz is only possible by using FM_ FAST.
  '''
  def set_operation_mode(self, modes):

  '''!
    @brief Get sensor operation mode
    @return Return the character string of the operation mode
  '''
  def get_operation_mode(self):

  '''!
    @brief Set the rate of obtaining geomagnetic data, the higher, the faster (without delay function)
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
    @brief Get the config data rate, unit: HZ
    @return rate
  '''
  def get_rate(self):

  '''!
    @brief Set preset mode, make it easier for users to configure sensor to get geomagnetic data (The default rate for obtaining geomagnetic data is 12.5Hz)
    @param modes 
    @n BMM350_PRESETMODE_LOWPOWER       Low power mode, get a fraction of data and take the mean value.
    @n BMM350_PRESETMODE_REGULAR        Regular mode, get a number of data and take the mean value.
    @n BMM350_PRESETMODE_ENHANCED       Enhanced mode, get a plenty of data and take the mean value.
    @n BMM350_PRESETMODE_HIGHACCURACY   High accuracy mode, get a huge number of data and take the mean value.

  '''
  def set_preset_mode(self, modes):

  '''!
    @brief Enable the measurement at x-axis, y-axis and z-axis, default to be enabled. After disabling, the geomagnetic data at x, y, and z axis are wrong.
    @param en_x
    @n   BMM350_X_EN        Enable the measurement at x-axis
    @n   BMM350_X_DIS       Disable the measurement at x-axis
    @param en_y
    @n   BMM350_Y_EN        Enable the measurement at y-axis
    @n   BMM350_Y_DIS       Disable the measurement at y-axis
    @param en_z
    @n   BMM350_Z_EN        Enable the measurement at z-axis
    @n   BMM350_Z_DIS       Disable the measurement at z-axis
  '''
  def set_measurement_XYZ(self, en_x = BMM350_X_EN, en_y = BMM350_Y_EN, en_z = BMM350_Z_EN):

  '''!
    @brief Get the enabling status at x-axis, y-axis and z-axis
    @return Return enabling status at x-axis, y-axis and z-axis as a character string
  '''
  def get_measurement_state_XYZ(self):

  '''!
    @brief Get the geomagnetic data of 3 axis (x, y, z)
    @return The list of the geomagnetic data at 3 axis (x, y, z) unit: uT
    @       [0] The geomagnetic data at x-axis
    @       [1] The geomagnetic data at y-axis
    @       [2] The geomagnetic data at z-axis
  '''
  def get_geomagnetic_data(self):

  '''!
    @brief Get compass degree
    @return Compass degree (0° - 360°)  0° = North, 90° = East, 180° = South, 270° = West.
  '''
  def get_compass_degree(self):

  '''!
    @brief Enable or disable data ready interrupt pin
    @n After enabling, the DRDY pin jump when there's data coming.
    @n After disabling, the DRDY pin will not jump when there's data coming.
    @n High polarity: active on high, the default is low level, which turns to high level when the interrupt is triggered.
    @n Low polarity: active on low, default is high level, which turns to low level when the interrupt is triggered.
    @param modes
    @n     BMM350_ENABLE_INTERRUPT        Enable DRDY
    @n     BMM350_DISABLE_INTERRUPT       Disable DRDY
    @param polarity
    @n     BMM350_ACTIVE_HIGH              High polarity
    @n     BMM350_ACTIVE_LOW               Low polarity
  '''
  def set_data_ready_pin(self, modes, polarity):

  '''!
    @brief Get data ready status, determine whether the data is ready
    @return status
    @n True  Data ready
    @n False Data is not ready
  '''
  def get_data_ready_state(self):

  '''!
    @brief Set threshold interrupt, an interrupt is triggered when the geomagnetic value of a channel is beyond/below the threshold
    @n      High polarity: active on high level, the default is low level, which turns to high level when the interrupt is triggered.
    @n      Low polarity: active on low level, the default is high level, which turns to low level when the interrupt is triggered.
    @param modes
    @n     LOW_THRESHOLD_INTERRUPT       Low threshold interrupt mode
    @n     HIGH_THRESHOLD_INTERRUPT      High threshold interrupt mode
    @param  threshold
    @n     Threshold, default to expand 16 times, for example: under low threshold mode, if the threshold is set to be 1, actually the geomagnetic data below 16 will trigger an interrupt
    @param polarity
    @n     POLARITY_HIGH      High polarity
    @n     POLARITY_LOW       Low polarity
  '''
  def set_threshold_interrupt(self, modes, threshold, polarity):

  '''!
    @brief Get the data that threshold interrupt occured
    @return Return the list for storing geomagnetic data, how the data at 3 axis influence interrupt status,
    @n      [0] The data triggering threshold at x-axis, when the data is NO_DATA, the interrupt is triggered.
    @n      [1] The data triggering threshold at y-axis, when the data is NO_DATA, the interrupt is triggered.
    @n      [2] The data triggering threshold at z-axis, when the data is NO_DATA, the interrupt is triggered.
  '''
  def get_threshold_data(self):
```

## History

- 2024/05/11 - Version 1.0.0 released.

## Credits

Written by [GDuang](yonglei.ren@dfrobot.com), 2024. (Welcome to our website)
