#include "DFRobot_BMM350.h"

#define SDA_PIN 8  
#define SCL_PIN 9 

struct MagnetReading {
    float x;
    float y;
    float z;
};

class Magnet {
    public:
        Magnet() 
        : bmm350(&Wire, 0x14)
        {
            Wire.begin(SDA_PIN, SCL_PIN);
            bmm350.begin();
            bmm350.setOperationMode(eBmm350NormalMode);
            bmm350.setPresetMode(BMM350_PRESETMODE_HIGHACCURACY,BMM350_DATA_RATE_25HZ);
            bmm350.setMeasurementXYZ();
        }

        void set_hard_iron_offset(float x, float y, float z) {
            hard_iron_offset[0] = x;
            hard_iron_offset[1] = y;
            hard_iron_offset[2] = z;
        }

        void set_soft_iron_matrix(float matrix[3][3]) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    soft_iron_matrix[i][j] = matrix[i][j];
                }
            }
        }

        MagnetReading read_calibrated_data() {
            sBmm350MagData_t sensor_mag_data = bmm350.getGeomagneticData();
            float mag_data[3];

            mag_data[0] = sensor_mag_data.float_x + hard_iron_offset[0];
            mag_data[1] = sensor_mag_data.float_y + hard_iron_offset[1];
            mag_data[2] = sensor_mag_data.float_z + hard_iron_offset[2];

            for (int i = 0; i < 3; i++) {
                mag_data[i] = (soft_iron_matrix[i][0] * mag_data[0]) + (soft_iron_matrix[i][1] * mag_data[1]) + (soft_iron_matrix[i][2] * mag_data[2]);
            }

            MagnetReading calibrated_data = { mag_data[0], mag_data[1], mag_data[2] };
            return calibrated_data;
        }

        float getCompassDegree(MagnetReading mag) {
            float compass = 0.0;
            compass = atan2(mag.x, mag.y);
            if (compass < 0) {
                compass += 2 * PI;
            }
            if (compass > 2 * PI) {
                compass -= 2 * PI;
            }
            return compass * 180 / M_PI;
        }

    private:
        DFRobot_BMM350_I2C bmm350;

        float hard_iron_offset[3] = { 23.71, 5.45, 8.27 };
        float soft_iron_matrix[3][3] = {
            { 1.017, -0.024, 0.023 },
            { -0.024, 0.994, 0.002 },
            { 0.023, 0.002, .991 }
        };

};
