#ifndef CHECKPOINT_WRITER_H
#define CHECKPOINT_WRITER_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include "toojpeg.h"
#include "TelemetryPoint.h"
#include <math.h>
#include <mutex>          // std::mutex, std::unique_lock

using namespace std;

class CheckpointWriter
{

public:
    static void checkpoint(int32_t width, int32_t height, TelemetryPoint scan_data[], int scan_size);
    static void advanceCheckpoint();

private:
	static int _checkpoint_num;
    static mutex jpeg_mutex;
    static ofstream jpeg;
    static const int channels = 3;
    static void writeJpegByte(unsigned char oneByte);
    static void addScanData(unsigned char *pixels, int width, int height, int x, int y, int r, int g, int b, int quality);
};

int CheckpointWriter::_checkpoint_num=0;
mutex CheckpointWriter::jpeg_mutex;
ofstream CheckpointWriter::jpeg;

void CheckpointWriter::writeJpegByte(unsigned char oneByte)
{
    jpeg << oneByte;
}

void CheckpointWriter::checkpoint(int32_t width, int32_t height, TelemetryPoint scan_data[], int scan_size){
	int scale_factor = 10;
	unique_lock<mutex> lock(jpeg_mutex);
	unsigned char *pixels = new unsigned char[width*height*channels];
	for(int32_t i = 0; i < height*width*channels; i= i+3){
			pixels[i] = 255;
			pixels[i+1] = 255;
			pixels[i+2] = 255;
	}
	char buffer [20];	

	for(int i = 0; i < scan_size; i++){
		addScanData(pixels, width, height, width/2 + scan_data[i].x/scale_factor, height/2 + (-1 * scan_data[i].y/scale_factor), 255, 0 ,0 , scan_data[i].quality);
	}
	addScanData(pixels, width, height, width/2, height/2, 0, 255 ,0 , 255);

	sprintf (buffer, "map_%d_%d.jpg", _checkpoint_num, scan_size);
	jpeg.open(buffer);
    TooJpeg::writeJpeg(writeJpegByte, pixels, width, height);
	jpeg.close();
}

void CheckpointWriter::advanceCheckpoint(){
	_checkpoint_num++;
}

void CheckpointWriter::addScanData(unsigned char *pixels, int width, int height, int x, int y, int r, int g, int b, int quality){
	int pad = 4;
	for(int yp = -pad; yp < pad; yp++) {
		for(int xp = -pad; xp < pad; xp++) {
			int pixel_num = (((y+yp)*width) + (x+xp))*channels;
			pixels[pixel_num] = r;
			pixels[pixel_num+1] = b;
			pixels[pixel_num+2] = g;
		}
	}
}

#endif