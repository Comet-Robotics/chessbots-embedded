#include "DFRobot_BMM350.h"
#include "robot/magnet.h"
#include "utils/logging.h"

#define SDA_PIN 8  
#define SCL_PIN 9

Magnet::Magnet()
    : bmm350(&Wire, 0x14)
{
    Wire.begin(SDA_PIN, SCL_PIN);
    int maxTries = 6;
    int errorCode = -1;
    while (maxTries-- > 0 && (errorCode=bmm350.begin()))
    {
        delay(500);
        serialLog("Retrying BMM350 connection... (error code ", 1);
        serialLog(errorCode, 1);
        serialLogln(")", 1);
    }
    if (errorCode > 0) {
        serialLogln("BMM350 not detected at default I2C address. Check wiring.", 1);
        return;
    } else {
        serialLogln("BMM350 detected.", 1);
    }
    bmm350.setOperationMode(eBmm350NormalMode);
    bmm350.setPresetMode(BMM350_PRESETMODE_HIGHACCURACY, BMM350_DATA_RATE_25HZ);
    bmm350.setMeasurementXYZ();
    activeFlag = true;
    // bmm350.setDataReadyPin(BMM350_ENABLE_INTERRUPT, BMM350_ACTIVE_LOW);
    // pinMode(/*Pin */ 13, INPUT_PULLUP);
    // maxTries = 5;
    // bool dataReady = false;
    // while (maxTries-- > 0 && !(dataReady=bmm350.getDataReadyState())) {
    //     delay(100);
    // }
    // if (dataReady) {
    //     serialLogln("BMM350 data ready interrupt enabled.", 1);
    //     activeFlag = true;
    // } else {
    //     serialLogln("BMM350 data ready interrupt not detected. Check wiring.", 1);
    // }
}

bool Magnet::isActive() {
    return activeFlag;
}

bool Magnet::isDataReady() {
    return bmm350.getDataReadyState();
}

void Magnet::set_hard_iron_offset(float x, float y, float z) {
    hard_iron_offset[0] = x;
    hard_iron_offset[1] = y;
    hard_iron_offset[2] = z;
}

void Magnet::set_soft_iron_matrix(float matrix[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            soft_iron_matrix[i][j] = matrix[i][j];
        }
    }
}

MagnetReading Magnet::read_calibrated_data() {
    sBmm350MagData_t sensor_mag_data = bmm350.getGeomagneticData();
    float hi_data[3];
    float mag_data[3];

    hi_data[0] = sensor_mag_data.float_x - hard_iron_offset[0];
    hi_data[1] = sensor_mag_data.float_y - hard_iron_offset[1];
    hi_data[2] = sensor_mag_data.float_z - hard_iron_offset[2];

    for (int i = 0; i < 3; i++) {
        mag_data[i] = (soft_iron_matrix[i][0] * hi_data[0]) + (soft_iron_matrix[i][1] * hi_data[1]) + (soft_iron_matrix[i][2] * hi_data[2]);
    }

    MagnetReading calibrated_data = { mag_data[0], mag_data[1], mag_data[2] };
    return calibrated_data;
}

float Magnet::getCompassDegree(MagnetReading mag) {
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

float Magnet::readDegreesRaw() {
    MagnetReading mag = read_calibrated_data();
    return getCompassDegree(mag);
}

float Magnet::readDegrees() {
    // // Only works if data ready interrupt is enabled
    // if (previousReading >= 0 && !bmm350.getDataReadyState()) {
    //     return previousReading; // Return the last reading if data is not ready and previousReading is valid
    // }
    float currentReading = readDegreesRaw();
    // Handle first reading
    if (previousReading < 0) {
        previousReading = currentReading;
        return currentReading;
    }
    // Handle wrap-around
    if (currentReading - previousReading > 180) {
        previousReading += 360;
    } else if (currentReading - previousReading < -180) {
        previousReading -= 360;
    }
    // Simple low-pass filter
    // currentReading = previousReading * 0.8 + currentReading * 0.2;
    if (currentReading >= 360) currentReading -= 360;
    else if (currentReading < 0) currentReading += 360;
    previousReading = currentReading;
    return currentReading;
}
