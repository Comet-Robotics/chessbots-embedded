#ifndef MAGNET_H
#define MAGNET_H

#include "Arduino.h"
#include "DFRobot_BMM350.h"

struct MagnetReading {
    float x;
    float y;
    float z;
};

class Magnet {
    public:
        Magnet();
        void set_hard_iron_offset(float x, float y, float z);
        void set_soft_iron_matrix(float matrix[3][3]);
        struct MagnetReading read_calibrated_data();
        float getCompassDegree(struct MagnetReading mag);
        float readDegreesRaw();
        float readDegrees();
        bool isDataReady();
        bool isActive();
    private:
        float hard_iron_offset[3] = { -10.20, -2.62, -13.21 };
        float soft_iron_matrix[3][3] = {
            { 1.024, -0.020, 0.027 },
            { -0.020, 0.968, 0.008 },
            { 0.027, 0.008, 1.011 }
        };
        DFRobot_BMM350_I2C bmm350;

        float previousReading = -1.0;
        bool activeFlag = false;
};

#endif // MAGNET_H