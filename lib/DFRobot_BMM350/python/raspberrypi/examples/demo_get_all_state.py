# -*- coding:utf-8 -*-
'''
  @file demo_get_all_state.py
  @brief Get all the config and self test status, the sensor can change from normal mode to sleep mode after soft reset
  @n Experimental phenomenon: the sensor config and self test information are printed in the serial port.
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

  print('Power mode after sensor initialization: ', bmm350.get_operation_mode()) 
  
  '''
    Sensor self test, the returned character string indicates the test result.
  '''
  print(bmm350.self_test())

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
    Get the config data rate unit: HZ
  '''
  print("rates is %d HZ"%bmm350.get_rate() ) 
  
  '''
    Get the character string of enabling status at x-axis, y-axis and z-axis
  '''
  print(bmm350.get_measurement_state_XYZ()) 
  
  '''
    @brief Soft reset, restore to suspended mode after soft reset. (You need to manually enter the normal mode)
  '''
  bmm350.soft_reset()
  print('Power mode after software reset: ', bmm350.get_operation_mode()) 
  
def loop():
  '''
    Get the operation mode character string of the sensor
  '''
  print('Current power mode: ', bmm350.get_operation_mode()) 
  exit()
  

if __name__ == "__main__":
  setup()
  while True:
    loop()
