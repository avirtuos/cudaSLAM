/*
 *  RPLIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2019 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "BMP.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms)
{
    while (ms >= 1000)
    {
        usleep(1000 * 1000);
        ms -= 1000;
    };
    if (ms != 0)
        usleep(ms * 1000);
}
#endif



#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
    if(drv != NULL)
    {
        drv->stop();
        drv->stopMotor();
        RPlidarDriver::DisposeDriver(drv);
        drv = NULL;
    }
}

int main(int argc, const char *argv[])
{
    const char *opt_com_path = NULL;
    _u32         baudrateArray[2] = {115200, 256000};
    _u32         opt_com_baudrate = 0;
    u_result     op_result;

    opt_com_path = argv[1];
    opt_com_baudrate = strtoul(argv[2], NULL, 10);


    BMP bmp2(2000, 2000);
    bmp2.fill_region(250, 1000, 10, 10, 255, 0, 0, 255);
    //bmp2.fill_region(0, 0, 50, 50, 0, 0, 255, 255);
    //bmp2.fill_region(150, 0, 100, 150, 0, 255, 0, 255);


    // create the driver instance
    RPlidarDriver *drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!drv)
    {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    rplidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    if(!drv)
        drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate)))
    {
        op_result = drv->getDeviceInfo(devinfo);

        if (IS_OK(op_result))
        {
            connectSuccess = true;
        }
        else
        {
            delete drv;
            drv = NULL;
        }
    }

    if (!connectSuccess)
    {

        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , opt_com_path);
        goto on_finished;
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
        goto on_finished;
    }

    signal(SIGINT, ctrlc);

    drv->startMotor();
    // start scan...
    drv->startScan(0, 1);


    // fetech result and print it out...
    while (1)
    {
        rplidar_response_measurement_node_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanData(nodes, count);

        if (IS_OK(op_result))
        {
            drv->ascendScanData(nodes, count);
            printf("count %d \n", count);

            for (int pos = 0; pos < (int)count ; ++pos)
            {
                float distance = nodes[pos].distance_q2 / 4.0f;
                float angle = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
                uint32_t x = 250 + roundf(sin (angle * PI / 180) * distance) / 3;
                uint32_t y = 1000 + roundf(cos (angle * PI / 180) * distance) / 3;
                uint32_t quality = 3 * round(nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
                bmp2.fill_region(x, y, 4, 4, quality, 0, 0, 255);
                printf("%s theta: %03.2f Dist: %08.2f Q: %d X: %d Y: %d\n",
                       (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S " : "  ",
                       angle,
                       distance,
                       quality,
                       x,
                       y);
            }
            bmp2.write("t1_24_copy.bmp");
            /*

               //     void set_pixel(uint32_t x, uint32_t y, uint8_t B, uint8_t G, uint8_t R, uint8_t A)
            //
            float op = (float)Math.Sin(angle) * distance;
            //Get CAH
            float ad = (float)Math.Cos(angle) * distance;
            //Add to old Vector
            return (CurPos + new Vector2(ad, op));
            */
        }

        if (ctrl_c_pressed)
        {
            break;
        }
    }

    drv->stop();
    drv->stopMotor();
    // done!
on_finished:
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return 0;
}

