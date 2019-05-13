#include "MapReader.cuh"

__host__ __device__
MapReader::MapReader(int32_t num_buckets, int32_t width, int32_t height, MapIndex *index, MapPoint *map){
	_num_buckets = num_buckets;
	_width = width;
	_height = height;
	_index = index;
	_map = map;
}

__host__ __device__
MapReader::~MapReader(){

}

__host__ __device__
int MapReader::getOccupancy(int x, int y){
	int32_t e_width = _width/2;
	int32_t e_height = _height/2;

	if(x > e_width || x < -1 * e_width || y > e_height || y < -1 * e_height){
		return 0;
	}

    int pos =  ((e_height + y) * _width)+ e_width + x;
    int index_pos = pos % _num_buckets;
    MapIndex *index = _index + index_pos;
	int map_offset = index->offset;

    for(int i =0; i < index->size; i++){
    	MapPoint *map = _map+map_offset+i;
    	if(map->x == x && map->y == y){
			return map->occupancy;
    	}
    }

    return 0;
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