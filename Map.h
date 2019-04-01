

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include "toojpeg.h"
#include "BMP.h"
#include <math.h>
#include <mutex>          // std::mutex, std::unique_lock

using namespace std;

class Map
{

public:
    Map(int32_t width_arg , int32_t height_arg);
    ~Map();
    void addScanData(int x, int y, int quality);
    void addScanData(int x, int y, int r, int g, int b, int quality);
    void flush();

private:
	static void writeJpegByte(unsigned char oneByte);
	int32_t width;
	int32_t height;
	int32_t channels=3;
   // BMP bmp;
    static ofstream jpeg;
    static mutex jpeg_mutex;
    unsigned char *pixels;
};

ofstream Map::jpeg;
mutex Map::jpeg_mutex;

Map::Map(int32_t width_arg, int32_t height_arg) {
	width=width_arg;
	height=height_arg;
	pixels = new unsigned char[width*height*channels];
	for(int32_t i = 0; i < height*width*channels; i++){
			pixels[i] = 255;
	}

	//addScanData(0,height/2,width/2,0,255,0);
	//addScanData(0,height-10,10,0,255,0);
	//addScanData(0,height-10,width-10,0,255,0);
	addScanData(0,1000,1000,0,255,0);
}

Map::~Map(){
	delete [] pixels;
}

void Map::addScanData(int x, int y, int quality){
	addScanData(x,y,255,0,0,quality);
}

void Map::addScanData(int x, int y, int r, int g, int b, int quality){
	int pad = 4;
	for(int yp = -pad; yp < pad; yp++) {
		for(int xp = -pad; xp < pad; xp++) {
			int pixel_num = (((y+yp)*width) + (x+xp))*channels;
			pixels[pixel_num] = r;
			pixels[pixel_num+1] = b;
			pixels[pixel_num+2] = g;
		}
	}
	//bmp.fill_region(x, y, 4, 4, quality, 0, 0, 255);
}

void Map::writeJpegByte(unsigned char oneByte)
{
    jpeg << oneByte;
}

void Map::flush(){
	//unique_lock<mutex> lock(jpeg_mutex);
	jpeg.open("map.jpg");
    TooJpeg::writeJpeg(writeJpegByte, pixels, width, height);
	//bmp.write("map.bmp");
	jpeg.close();
}

