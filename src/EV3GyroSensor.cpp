#include "EV3GyroSensor.h"

const static char *TAG = "EV3GyroSensor";

void EV3GyroSensor::messageHandler(uint8_t mode, uint8_t *message, int length)
{
    switch (mode)
    {
    //TODO: int16, nicht uint16!
    case (uint8_t) EV3GyroSensorMode::GYRO_ANG:
        if (onGyroAng)
            onGyroAng(((unsigned short)message[1] << 8) | (unsigned char)message[0]);
        break;
    case (uint8_t) EV3GyroSensorMode::GYRO_RATE:
        if (onGyroRate)
            onGyroRate(((unsigned short)message[1] << 8) | (unsigned char)message[0]);
        break;
    default:
        ESP_LOGE(TAG, "Currently not supported EV3 gyro sensor mode %d", mode);
    }
}