#include "EV3UltrasonicSensor.h"

const static char *TAG = "EV3UltrasonicSensor";

void EV3UltrasonicSensor::messageHandler(uint8_t mode, uint8_t *message, int length)
{
    switch (mode)
    {
    case (uint8_t) EV3UltrasonicSensorMode::US_DIST_CM:
        if (onDistCm)
            onDistCm(((unsigned short)message[1] << 8) | (unsigned char)message[0]);
        break;
    case (uint8_t) EV3UltrasonicSensorMode::US_DIST_IN:
        if (onDistIn)
            onDistIn(((unsigned short)message[1] << 8) | (unsigned char)message[0]);
        break;
    case (uint8_t) EV3UltrasonicSensorMode::US_LISTEN:
        if (onListen)
            onListen(message[0]);
        break;
    default:
        ESP_LOGE(TAG, "Currently not supported EV3 Ultrasonic Sensor mode %d", mode);
    }
}