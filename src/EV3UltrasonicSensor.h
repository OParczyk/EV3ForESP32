#include "EV3SensorPort.h"

#ifndef EV3UltrasonicSensor_h
#define EV3UltrasonicSensor_h

/**
 *  Available modes of the EV3 ultrasonic sensor.
 */
enum struct EV3UltrasonicSensorMode : uint8_t
{
    US_DC_IN = 6,
    US_DC_CM = 5,
    US_SI_IN = 4,
    US_SI_CM = 3,
    US_LISTEN = 2,
    US_DIST_IN = 1,
    US_DIST_CM = 0
};

/**
 * Handler of the EV3 Ultrasonic sensor.
 */
class EV3UltrasonicSensor
{
private:
    EV3SensorPort *_port;

    std::function<void(ushort)> onDistIn;
    std::function<void(ushort)> onDistCm;
    std::function<void(byte)> onListen;

    /**
     * Handler for single messages from the EV3 sensor port
     */
    void messageHandler(uint8_t mode, uint8_t *message, int length);

public:
    EV3UltrasonicSensor(EV3SensorPort *port) : _port(port)
    {
        _port->setMessageHandler([this](uint8_t mode, uint8_t *message, int length) { this->messageHandler(mode, message, length); });
    }

    /**
     * Set the mode of the ultrasonic sensor
     */
    void setMode(EV3UltrasonicSensorMode mode)
    {
        _port->selectSensorMode(static_cast<uint8_t>(mode));
    }

    /**
     * Set the handler for DIST_IN messages
     */
    void setOnDistIn(std::function<void(ushort)> h)
    {
        this->onDistIn = h;
    }

    /**
     * Set the handler for DIST_CM messages
     */
    void setOnDistCm(std::function<void(ushort)> h)
    {
        this->onDistCm = h;
    }

    /**
     * Set the handler for LISTEN messages
     */
    void setOnListen(std::function<void(byte)> h)
    {
        this->onListen = h;
    }

};
#endif