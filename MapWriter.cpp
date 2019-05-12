#include "MapWriter.h"


MapWriter::MapWriter(int32_t num_buckets, int32_t width, int32_t height){
	_num_buckets = num_buckets;
	_width = width;
	_height = height;
	_bucket_size = ceil(((float)width*(float)height)/((float)num_buckets));
	int mem_size = _bucket_size * num_buckets;
	_index = (MapIndex*) malloc(sizeof(MapIndex) * _num_buckets);
	_map = (MapPoint*) malloc(sizeof(MapPoint) * _bucket_size * _num_buckets);

	for(int i = 0; i < _num_buckets; i++){
		MapIndex *index = _index + i;
		index->size = 0;
		index->offset = 0;
	}
}

int MapWriter::getIndexSizeBytes(){
	return _num_buckets * sizeof(MapIndex);
}

int MapWriter::getMapSizeBytes(){
	int num_points = 0;
	for(int i = 0; i < _num_buckets; i++){
		MapIndex *index = _index + i;
		num_points += index->size;
	}
	return num_points * sizeof(MapPoint);
}

void MapWriter::dump(int count){
	for(int j = 0; j < _num_buckets; j++){
		MapIndex *index = _index + j;
		int map_offset = (j * _bucket_size);
		for(int i =0; i < index->size; i++){
	    	MapPoint *map = _map+map_offset+i;
	    	printf("POINT(%d): x: %d y: %d occ: %d \n", count, map->x, map->y, map->occupancy);
	    	printf("%d %d %d\n", map->x, map->y, count);
	    }
	}
}

void MapWriter::addPoint(int16_t x, int16_t y){
	int32_t e_width = _width/2;
	int32_t e_height = _height/2;

	if(x > e_width || x < -1 * e_width || y > e_height || y < -1 * e_height){
		char buffer [64];
		sprintf(buffer, "Point x: %d y: %d is outside map bounds", x, y);
		throw buffer;
	}

    int pos =  ((e_height + y) * _width)+ e_width + x;
    int index_pos = pos % _num_buckets;
    MapIndex *index = _index + index_pos;
	int map_offset = (index_pos * _bucket_size);

    bool found = false;
    for(int i =0; i < index->size && !found; i++){
    	MapPoint *map = _map+map_offset+i;
    	if(map->x == x && map->y == y){
			map->occupancy++;
			found = true;
    	}
    }

    if(!found){
	    MapPoint *map = _map + map_offset + index->size;
	    index->size++;
	    map->x = x;
	    map->y = y;
	    map->occupancy = 1;
	}
}

MapWriter::~MapWriter(){
	free(_index);
	free(_map);
}