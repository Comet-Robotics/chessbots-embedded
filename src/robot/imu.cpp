//using magnetometer data from IMU to compute positioning of bot
/*Tasks:

1. Create IMU object

2. Get raw magnetometer coordinates (in rad) and store it as a xyzFloat structure

3. Define computeHeading()
    3.1   Compute arctangent of coordinates
    3.2   Convert from radians to degrees
    3.3   Validate the value found in 3.2 to lie b/w 0 to 360
    3.4   Return heading
    
*/

#include <iostream>
#include <Wire.h>
#include <math.h>
#include "ICM20948_WE.h"

using namespace std; 

ICM20948_WE botIMU = ICM20948_WE(Wire);

/*struct Coords{
    float x_val;
    float y_val;
    float z_val;
};
*/

float computeHeading(float x, float y){
    float heading = atan2(-x, y) * (180.0/M_PI);
    if (heading < 0){
        heading += 360;
    }
    return heading;
}

void loop(){
xyzFloat coordinates;
botIMU.getMagValues(&coordinates);

float orientation = computeHeading(coordinates.x, coordinates.y);
cout << orientation << endl;

}