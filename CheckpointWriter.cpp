#include "CheckpointWriter.h"

int CheckpointWriter::_checkpoint_num=0;
mutex CheckpointWriter::jpeg_mutex;
ofstream CheckpointWriter::jpeg;

void CheckpointWriter::writeJpegByte(unsigned char oneByte)
{
    jpeg << oneByte;
}

void CheckpointWriter::checkpoint(const string prefix, int32_t width, int32_t height, TelemetryPoint scan_data[], int scan_size){
	int scale_factor = 1;
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

	sprintf (buffer, "map_%s_%d_%d.jpg",prefix.c_str(), _checkpoint_num, scan_size);
	jpeg.open(buffer);
    TooJpeg::writeJpeg(writeJpegByte, pixels, width, height);
	jpeg.close();
}

void CheckpointWriter::checkpoint(const string prefix, int32_t width, int32_t height, TelemetryPoint scan_data[], int scan_size, MapPoint map[], LocalizedOrigin *location){
	int scale_factor = 1;
	unique_lock<mutex> lock(jpeg_mutex);
	unsigned char *pixels = new unsigned char[width*height*channels];
	for(int32_t i = 0; i < height*width*channels; i= i+3){
		pixels[i] = 255;
		pixels[i+1] = 255;
		pixels[i+2] = 255;
	}

	for(int32_t i = 0; i < height*width; i++){
		if(map[i].occupancy > 0) {
			int x = i % width;
			int y = height - ((i - x)/height);
			int r = 0;
			int g = 0;
			int b = 0;
			if(map[i].occupancy > 200){
				r = map[i].occupancy;
			} else if (map[i].occupancy > 100){
				g = map[i].occupancy;
			} else {
				b = map[i].occupancy;
			}
			addScanData(pixels, width, height, x, y, r, g ,b , 255, 8);
		}
	}

	for(int i = 0; i < scan_size; i++){
		//addScanData(pixels, width, height, width/2 + scan_data[i].x/scale_factor, height/2 + (-1 * scan_data[i].y/scale_factor), 155, 155 ,0 , 255);
	}

	addScanData(pixels, width, height, width/2 + location->x_offset, height/2 + location->y_offset, 0, 255 ,0 , 255, 4);

	char buffer [20];	
	sprintf (buffer, "map_%s_%d_%d.jpg",prefix.c_str(), _checkpoint_num, scan_size);
	jpeg.open(buffer);
    TooJpeg::writeJpeg(writeJpegByte, pixels, width, height);
	jpeg.close();
}

void CheckpointWriter::advanceCheckpoint(){
	_checkpoint_num++;
}

void CheckpointWriter::addScanData(unsigned char *pixels, int width, int height, int x, int y, int r, int g, int b, int quality,int pad){
	for(int yp = -pad; yp < pad; yp++) {
		for(int xp = -pad; xp < pad; xp++) {
			int pixel_num = (((y+yp)*width) + (x+xp))*channels;
			pixels[pixel_num] = r;
			pixels[pixel_num+1] = b;
			pixels[pixel_num+2] = g;
		}
	}
}