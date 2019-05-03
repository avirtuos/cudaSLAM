
#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include "TelemetryPoint.h"
#include <math.h>


#define PI 3.14159265

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;

class LaserScan
{

public:
    LaserScan(const char *com_port_arg, _u32 baudrate_arg);
    int scan(TelemetryPoint result_buffer[], const int buffer_length);
    bool start();
    bool stop();
    bool connect();
    TelemetryPoint *scan();


private:
    bool checkRPLIDARHealth(RPlidarDriver *drv);
    float getAngle(const rplidar_response_measurement_node_hq_t &node);
    bool is_connected;
    const char *com_port;
    _u32 baudrate;
    RPlidarDriver *drv;
    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t node_count;

};

LaserScan::LaserScan(const char *com_port_arg, _u32 baudrate_arg)
{
    com_port = com_port_arg;
    baudrate = baudrate_arg;
    is_connected = false;
    drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    node_count = _countof(nodes);
    if (!drv)
    {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }
}


int LaserScan::scan(TelemetryPoint result_buffer[], const int buffer_length)
{
    int result_size = 0;
    TelemetryPoint *cur = result_buffer;
    u_result op_result = drv->grabScanDataHq(nodes, node_count);
    if (IS_OK(op_result))
    {
        drv->ascendScanData(nodes, node_count);

        for (int pos = 0; pos < (int)node_count && result_size < buffer_length; ++pos)
        {
            int16_t quality = round(nodes[pos].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            if(quality > 0)
            {
                //convert to centi-meters by dividing by 10, millimeter resolution isn't useful
                float distance = (nodes[pos].dist_mm_q2 / 4.0f)/10;
                float angle = (getAngle(nodes[pos]) * 3.14159265 / 180);
                int16_t x = roundf(sin (angle) * distance);
                int16_t y = roundf(cos (angle) * distance);

                //dedup points that are essentailly identical after we converted to centimeters, this reduces calculation costs
                //later and allows fo even naive scoring algos to work well without worrying about deduping.
                if(result_size < 1 || result_buffer[result_size-1].x != x || result_buffer[result_size-1].y != y){
                    cur->x = x;
                    cur->y = y;
                    cur->quality = quality;
                    cur->distance = distance;
                    cur->angle = angle;

                    result_size++;
                    cur = result_buffer + result_size;
                }
            }
        }
    }

    return result_size;
}

bool LaserScan::stop()
{
    is_connected = false;
    if(drv != NULL)
    {
        drv->stop();
        drv->stopMotor();
        RPlidarDriver::DisposeDriver(drv);
        drv = NULL;
    }
    return true;
}

bool LaserScan::start()
{
    u_result op_result;
    rplidar_response_device_info_t devinfo;

    if(!drv)
        drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

    if (IS_OK(drv->connect(com_port, baudrate)))
    {
        op_result = drv->getDeviceInfo(devinfo);

        if (!IS_OK(op_result))
        {
            delete drv;
            drv = NULL;
            fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n", com_port);
            return false;
        }
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ; ++pos)
    {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
           "Firmware Ver: %d.%02d\n"
           "Hardware Rev: %d\n"
           , devinfo.firmware_version >> 8
           , devinfo.firmware_version & 0xFF
           , (int)devinfo.hardware_version);

    // check health...
    if (!checkRPLIDARHealth(drv))
    {
        return false;
    }

    drv->startMotor();
    // start scan...
    drv->startScan(0, 1);

    is_connected = true;
    return true;
}

bool LaserScan::checkRPLIDARHealth(RPlidarDriver *drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result))   // the macro IS_OK is the preperred way to judge whether the operation is succeed.
    {
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR)
        {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        }
        else
        {
            return true;
        }

    }
    else
    {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}


float LaserScan::getAngle(const rplidar_response_measurement_node_hq_t &node)
{
    return node.angle_z_q14 * 90.f / 16384.f;
}

