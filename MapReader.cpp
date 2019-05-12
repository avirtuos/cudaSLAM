#include "MapReader.h"

MapReader::MapReader(int32_t num_buckets, int32_t width, int32_t height, MapIndex *index, MapPoint *map){
	_num_buckets = num_buckets;
	_width = width;
	_height = height;
	_index = index;
	_map = map;
}

MapReader::~MapReader(){

}

MapPoint* MapReader::getMap(){
	return _map;
}

int MapReader::getMapSize(){
	int num_points = 0;
	for(int i = 0; i < _num_buckets; i++){
		MapIndex *index = _index + i;
		num_points += index->size;
	}
	return num_points;
}