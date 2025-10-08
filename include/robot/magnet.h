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
        bool isDataReady() { return bmm350.getDataReadyState(); }
    private:
        float hard_iron_offset[3] = { -23.71, -5.45, -8.27 };
        float soft_iron_matrix[3][3] = {
            { 1.017, -0.024, 0.023 },
            { -0.024, 0.994, 0.002 },
            { 0.023, 0.002, .991 }
        };
        DFRobot_BMM350_I2C bmm350;

        float previousReading = 0.0;
};

#endif // MAGNET_H