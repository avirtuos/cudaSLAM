#ifndef TELEMETRY_POINT_H
#define TELEMETRY_POINT_H

struct TelemetryPoint
{
    int16_t x;
    int16_t y;
    //int16_t quality;
    float distance;
    float angle;
};

#endif