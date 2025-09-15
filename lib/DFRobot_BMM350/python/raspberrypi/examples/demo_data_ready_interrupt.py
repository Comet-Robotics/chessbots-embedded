# -*- coding:utf-8 -*-
'''
  @file demo_data_ready_interrupt.py
  @brief Data ready interrupt, DRDY interrupt will be triggered when the geomagnetic data is ready (the software and hardware can determine whether the interrupt occur)
  @n Experimental phenomenon: serial print the geomagnetic data at x-axis, y-axis and z-axis, unit (uT)
  @n Experimental phenomenon: the main controller interrupt will be triggered by level change caused by DRDY pin interrupt, then the geomagnetic data can be obtained.
  @n Connect the sensor DADY pin to the interrupt pin (RASPBERR_PIN_DRDY) of the main controller
  @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author      [GDuang](yonglei.ren@dfrobot.com)
  @version     V1.0.0
  @date        2024-05-11
  @url https://github.com/DFRobot/DFRobot_BMM350
'''
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from DFRobot_bmm350 import *

'''
  If you want to use I2C to drive this module, uncomment the codes below, and connect the module with Raspberry Pi via I2C port
  Connect to VCC，GND，SCL，SDA pin
'''
I2C_BUS         = 0x01   #default use I2C1
bmm350 = DFRobot_bmm350_I2C(I2C_BUS, 0x14)

def setup():
  while BMM350_CHIP_ID_ERROR == bmm350.sensor_init():
    print("sensor init error, please check connect") 
    time.sleep(1)

  '''
    Set sensor operation mode
      opMode:
        BMM350_SUSPEND_MODE      suspend mode: Suspend mode is the default power mode of BMM350 after the chip is powered, Current consumption in suspend mode is minimal, 
                                 so, this mode is useful for periods when data conversion is not needed. Read and write of all registers is possible.
        BMM350_NORMAL_MODE       normal mode: Get geomagnetic data normally.      
        BMM350_FORCED_MODE       forced mode: Single measurement, the sensor restores to suspend mode when the measurement is done.
        BMM350_FORCED_MODE_FAST  To reach ODR = 200Hz is only possible by using FM_ FAST.
  '''
  bmm350.set_operation_mode(BMM350_NORMAL_MODE)

  '''
    Get the operation mode character string of the sensor
  '''
  print('Current power mode: ', bmm350.get_operation_mode()) 

  '''
    Set preset mode, make it easier for users to configure sensor to get geomagnetic data (The default rate for obtaining geomagnetic data is 12.5Hz)
      presetMode:
        BMM350_PRESETMODE_LOWPOWER       Low power mode, get a small number of data and draw the mean value.
        BMM350_PRESETMODE_REGULAR        Regular mode, get a number of data and draw the mean value.
        BMM350_PRESETMODE_ENHANCED       Enhanced mode, get a large number of data and draw the mean value.
        BMM350_PRESETMODE_HIGHACCURACY   High accuracy mode, get a huge number of data and draw the mean value.
      rate:
        BMM350_DATA_RATE_1_5625HZ
        BMM350_DATA_RATE_3_125HZ
        BMM350_DATA_RATE_6_25HZ
        BMM350_DATA_RATE_12_5HZ  (default rate)
        BMM350_DATA_RATE_25HZ
        BMM350_DATA_RATE_50HZ
        BMM350_DATA_RATE_100HZ
        BMM350_DATA_RATE_200HZ
        BMM350_DATA_RATE_400HZ
  '''
  bmm350.set_preset_mode(BMM350_PRESETMODE_HIGHACCURACY,BMM350_DATA_RATE_25HZ)

  
  '''
    Enable the measurement at x-axis, y-axis and z-axis, default to be enabled, no config required. When disabled, the geomagnetic data at x, y, and z will be inaccurate.
    Refer to readme file if you want to configure more parameters.
  '''
  bmm350.set_measurement_XYZ()

  '''
    Enable or disable data ready interrupt pin
    After enabling, the pin DRDY signal jump when there's data coming.
    After disabling, the pin DRDY signal does not jump when there's data coming.
    High polarity: active on high, the default is low level, which turns to high level when the interrupt is triggered.
    Low polarity: active on low, the default is high level, which turns to low level when the interrupt is triggered.
      modes:
        BMM350_ENABLE_INTERRUPT    Enable DRDY
        BMM350_DISABLE_INTERRUPT   Disable DRDY
      polarity:
        BMM350_ACTIVE_HIGH  High polarity
        BMM350_ACTIVE_LOW   Low polarity
  '''
  bmm350.set_data_ready_pin(BMM350_ENABLE_INTERRUPT, BMM350_ACTIVE_LOW)
  

def loop():
  '''
    Get data ready status, determine whether the data is ready (through software)
      status:
        True    Data ready
        False   Data is not ready yet
  '''
  if bmm350.get_data_ready_state() == 1:
    rslt = bmm350.get_geomagnetic_data()
    print("mag x = %d ut"%rslt[0]) 
    print("mag y = %d ut"%rslt[1]) 
    print("mag z = %d ut"%rslt[2]) 
    print("") 
  else:
    time.sleep(1)


if __name__ == "__main__":
  setup()
  while True:
    loop()
