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

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include "LaserScan.h"

using namespace std;
using namespace std::chrono;

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}



int main(int argc, const char *argv[])
{
    bool *map = (bool *)malloc(30000 * 30000 * sizeof(bool));
    const char *com_path = argv[1];
    const _u32 com_baudrate = strtoul(argv[2], NULL, 10);

    LaserScan laser(com_path, com_baudrate);
    laser.start();

    signal(SIGINT, ctrlc);

    while(true)
    {
        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        laser.scan();
        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>( t2 - t1 ).count();

        cout << "Scan Duration: " << duration << " ms" << endl;
        if (ctrl_c_pressed)
        {
            laser.stop();
            break;
        }
    }


    return 0;
}

