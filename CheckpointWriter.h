#ifndef CHECKPOINT_WRITER_H
#define CHECKPOINT_WRITER_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include "toojpeg.h"
#include "TelemetryPoint.h"
#include "MapPoint.h"
#include <math.h>
#include <mutex>          // std::mutex, std::unique_lock

using namespace std;

class CheckpointWriter
{

public:
    static void checkpoint(const string prefix,int32_t width, int32_t height, TelemetryPoint scan_data[], int scan_size);
    static void checkpoint(const string prefix, int32_t width, int32_t height, TelemetryPoint scan_data[], int scan_size, MapPoint map[]);
    static void advanceCheckpoint();

private:
	static int _checkpoint_num;
    static mutex jpeg_mutex;
    static ofstream jpeg;
    static const int channels = 3;
    static void writeJpegByte(unsigned char oneByte);
    static void addScanData(unsigned char *pixels, int width, int height, int x, int y, int r, int g, int b, int quality, int pad);
};

#endif