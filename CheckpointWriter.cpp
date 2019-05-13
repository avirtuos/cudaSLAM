#include "CheckpointWriter.h"

int CheckpointWriter::_checkpoint_num=0;
mutex CheckpointWriter::jpeg_mutex;
ofstream CheckpointWriter::jpeg;

void CheckpointWriter::writeJpegByte(unsigned char oneByte)
{
    jpeg << oneByte;
}

void CheckpointWriter::checkpoint(const string prefix, int32_t width, int32_t height, TelemetryPoint scan_data[], int scan_size, MapPoint map[], LocalizedOrigin *location){
	int scale_factor = 3;
	unique_lock<mutex> lock(jpeg_mutex);
	int32_t img_width = width/scale_factor;
	int32_t img_height = height/scale_factor;
	unsigned char *pixels = new unsigned char[img_width*img_height*channels];
	for(int32_t i = 0; i < img_height*img_width*channels; i= i+3){
		pixels[i] = 255;
		pixels[i+1] = 255;
		pixels[i+2] = 255;
	}

	for(int32_t i = 0; i < height*width; i++){
		if(map[i].occupancy > 0) {
			int x = i % width;
			int y = height - ((i - x)/height) - 1;

			int r = 255;
			int g = 0;
			int b = 0;

			if(map[i].occupancy >=9){
				r = 0;
			    g = 0;
				b = 0;
			} else if(map[i].occupancy >= 6){
				r = 0;
				g = 0;
				b = 255;
			}
			
			addScanData(pixels, img_width, img_height, x/scale_factor, y/scale_factor, r, g ,b , 255, 2);
		}
	}

	for(int i = 0; i < scan_size; i++){
		addScanData(pixels, img_width, img_height, (width/2 + scan_data[i].x + location->x_offset)/scale_factor, (height/2 + (-1 * (scan_data[i].y+ location->y_offset)))/scale_factor
			, 0, 0 ,255 , 255, 1);
	}

	addScanData(pixels, img_width, img_height, (width/2 + location->x_offset)/scale_factor, (height/2 - location->y_offset)/scale_factor, 255, 128 ,30 , 255, 8);

	char buffer [40];	
	sprintf (buffer, "./maps/map_%s_%d_%d.jpg",prefix.c_str(), _checkpoint_num, scan_size);
	jpeg.open(buffer);
    TooJpeg::writeJpeg(writeJpegByte, pixels, img_width, img_height);
	jpeg.close();
}


void CheckpointWriter::checkpoint(const string prefix, int32_t width, int32_t height, TelemetryPoint scan_data[], int scan_size, int map_size, MapPoint *map, LocalizedOrigin *location){
	int scale_factor = 3;
	unique_lock<mutex> lock(jpeg_mutex);
	int32_t e_width = width/2;
	int32_t e_height = height/2;
	int32_t img_width = width/scale_factor;
	int32_t img_height = height/scale_factor;
	unsigned char *pixels = new unsigned char[img_width*img_height*channels];
	
	for(int32_t i = 0; i < img_height*img_width*channels; i= i+3){
		pixels[i] = 255;
		pixels[i+1] = 255;
		pixels[i+2] = 255;
	}

	for(int i = 0; i < map_size; i++){
		MapPoint *cur_point = map+i;
		if(cur_point->occupancy > 0) {

			int r = 255;
			int g = 0;
			int b = 0;

			if(cur_point->occupancy >=6){
				r = 0;
			    g = 0;
				b = 0;
			} else if(cur_point->occupancy >= 3){
				r = 0;
				g = 0;
				b = 255;
			}
			addScanData(pixels, img_width, img_height, (e_width+ cur_point->x)/scale_factor, (e_height - cur_point->y)/scale_factor, r, g ,b , 255, 4);
		}
	}

	for(int i = 0; i < scan_size; i++){
		addScanData(pixels, img_width, img_height, (e_width + scan_data[i].x + location->x_offset)/scale_factor, (e_height + (-1 * (scan_data[i].y+ location->y_offset)))/scale_factor
			, 0, 0 ,255 , 255, 1);
	}

	addScanData(pixels, img_width, img_height, (e_width + location->x_offset)/scale_factor, (e_height - location->y_offset)/scale_factor, 64, 128 ,255 , 255, 8);

	char buffer [40];	
	sprintf (buffer, "./maps/map_%s_%d_%d.jpg",prefix.c_str(), _checkpoint_num, scan_size);
	jpeg.open(buffer);
    TooJpeg::writeJpeg(writeJpegByte, pixels, img_width, img_height);
	jpeg.close();
}

void CheckpointWriter::advanceCheckpoint(){
	_checkpoint_num++;
}

void CheckpointWriter::addScanData(unsigned char *pixels, int width, int height, int x, int y, int r, int g, int b, int quality,int pad){
	for(int yp = -pad; yp < pad; yp++) {
		for(int xp = -pad; xp < pad; xp++) {
			int pixel_num = (((y+yp)*width) + (x+xp))*channels;
			if(y+yp >= height || y+yp < 0 || x+xp >= width || x+xp < 0){
				printf("OOB %d %d\n", x, y);
				continue;
			}
			
			pixels[pixel_num] = r;
			pixels[pixel_num+1] = b;
			pixels[pixel_num+2] = g;
		}
	}
}