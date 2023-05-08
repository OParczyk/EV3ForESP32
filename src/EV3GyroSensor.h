#include "EV3SensorPort.h"

#ifndef EV3GyroSensor_h
#define EV3GyroSensor_h

/**
 *  Available num_modes of the EV3 gyroscopic sensor.
 */
enum struct EV3GyroSensorMode : uint8_t
{
    TILT_ANG = 6,
    TILT_RATE = 5,
    GYRO_CAL = 4,
    GYRO_G_AND_A = 3,
    GYRO_FAS = 2,
    GYRO_RATE = 1,
    GYRO_ANG = 0
};

/**
 * Handler of the EV3 Gyroscopic sensor.
 */
class EV3GyroSensor
{
private:
    EV3SensorPort *_port;

    std::function<void(int16_t)> onGyroRate;
    std::function<void(int16_t)> onGyroAng;

    /**
     * Handler for single messages from the EV3 sensor port
     */
    void messageHandler(uint8_t mode, uint8_t *message, int length);

public:
    EV3GyroSensor(EV3SensorPort *port) : _port(port)
    {
        _port->setMessageHandler([this](uint8_t mode, uint8_t *message, int length) { this->messageHandler(mode, message, length); });
    }

    /**
     * Set the mode of the gyroscopic sensor
     */
    void setMode(EV3GyroSensorMode mode)
    {
        _port->selectSensorMode(static_cast<int8_t>(mode));
    }

    /**
     * Set the handler for GYRO_RATE messages
     */
    void setOnGyroRate(std::function<void(int16_t)> h)
    {
        this->onGyroRate = h;
    }

    /**
     * Set the handler for GYRO_ANG messages
     */
    void setOnGyroAng(std::function<void(uint16_t)> h)
    {
        this->onGyroAng = h;
    }

};
#endif