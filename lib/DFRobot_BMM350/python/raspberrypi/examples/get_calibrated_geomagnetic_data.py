# -*- coding:utf-8 -*-
'''
  @file get_calibrated_geomagnetic_data.py
  @brief Get the geomagnetic data at 3 axis (x, y, z), get the compass degree
  @n "Compass Degree", the angle formed when the needle rotates counterclockwise from the current position to the true north
  @n Experimental phenomenon: serial print the geomagnetic data at x-axis, y-axis and z-axis and the angle formed when the needle rotates counterclockwise from the current position to the true north
  @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author      [GDuang](yonglei.ren@dfrobot.com)
  @version     V1.0.0
  @date        2024-05-11
  @url https://github.com/DFRobot/DFRobot_BMM350
'''
import sys
import os
import math
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from DFRobot_bmm350 import *

'''
  If you want to use I2C to drive this module, uncomment the codes below, and connect the module with Raspberry Pi via I2C port
  Connect to VCC,GND，SCL,SDA pin
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
  
def loop():
  geomagnetic = bmm350.get_geomagnetic_data()
  #hard iron calibration parameters
  hard_iron= (-13.45, -28.95, 12.69 )
  #soft iron calibration parameters
  soft_iron= [
  ( 0.992, -0.006, -0.007 ),
  ( -0.006, 0.990, -0.004 ),
  ( -0.007, -0.004, 1.019 )
  ]

  # hard iron calibration
  geomagnetic[0] =geomagnetic[0] + hard_iron[0]
  geomagnetic[1] =geomagnetic[1] + hard_iron[1]
  geomagnetic[2] = geomagnetic[2] + hard_iron[2]

  #soft iron calibration
  for i in range(3):
    geomagnetic[i] = (soft_iron[i][0] * geomagnetic[0]) + (soft_iron[i][1] * geomagnetic[1]) + (soft_iron[i][2] * geomagnetic[2])
    
  compass = math.atan2(geomagnetic[0], geomagnetic[1])
  if compass < 0:
    compass += 2 * PI
  if compass > 2 * PI:
    compass -= 2 * PI
  degree=compass * 180 / M_PI
  print("---------------------------------")
  print("mag x = %.2f ut"%geomagnetic[0])
  print("mag y = %.2f ut"%geomagnetic[1])
  print("mag z = %.2f ut"%geomagnetic[2])
  print("---------------------------------")
  print("the angle between the pointing direction and north (counterclockwise) is: %.2f "%degree) 
  time.sleep(1)

if __name__ == "__main__":
  setup()
  while True:
    loop()
