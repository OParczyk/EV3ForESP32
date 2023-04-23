#include "EV3ColorSensor.h"

const static char *TAG = "EV3ColorSensor";

void EV3ColorSensor::messageHandler(uint8_t mode, uint8_t *message, int length)
{
    switch (mode)
    {
    case (uint8_t) EV3ColorSensorMode::COL_REFLECT:
        if (onColReflect)
            onColReflect(message[0]);
        break;
    case (uint8_t) EV3ColorSensorMode::COL_AMBIENT:
        if (onColAmbient)
            onColAmbient(message[0]);
        break;
    case (uint8_t) EV3ColorSensorMode::COL_COLOR:
        if (onColColor)
            onColColor(static_cast<EV3ColorSensorColor>(message[0]));
        break;
    case (uint8_t) EV3ColorSensorMode::RGB_RAW:
        if (onRGBRaw){
            EV3ColorSensorRGB rgb;
            rgb.r = ((unsigned short)message[1] << 8) | (unsigned char)message[0];
            rgb.g = ((unsigned short)message[3] << 8) | (unsigned char)message[2];
            rgb.b = ((unsigned short)message[5] << 8) | (unsigned char)message[4];
            
            onRGBRaw(rgb);
        }
        break;
    default:
        ESP_LOGE(TAG, "Currently not supported EV3 color sensor mode %d", mode);
    }
}

void writeEV3ColorSensorColorToStream(EV3ColorSensorColor col, Stream *stream)
{
    switch (col)
    {
    case EV3ColorSensorColor::NONE:
        stream->print("none");
        break;
    case EV3ColorSensorColor::BLACK:
        stream->print("black");
        break;
    case EV3ColorSensorColor::BLUE:
        stream->print("blue");
        break;
    case EV3ColorSensorColor::GREEN:
        stream->print("green");
        break;
    case EV3ColorSensorColor::YELLOW:
        stream->print("yellow");
        break;
    case EV3ColorSensorColor::RED:
        stream->print("red");
        break;
    case EV3ColorSensorColor::WHITE:
        stream->print("white");
        break;
    case EV3ColorSensorColor::BROWN:
        stream->print("brown");
        break;
    default:
        ESP_LOGE(TAG, "Unknown color %d", static_cast<uint8_t>(col));
        stream->print("unknown");
    }
}