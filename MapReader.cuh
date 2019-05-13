#ifndef MAP_READER_H
#define MAP_READER_H

#include "MapIndex.h"
#include "MapPoint.h"

using namespace std;

class MapReader
{

public:
    __host__ __device__
    MapReader(int32_t num_buckets, int32_t width, int32_t height, MapIndex *index, MapPoint *map);
    __host__ __device__
    ~MapReader();
    __host__ __device__
    int getOccupancy(int x, int y);
    MapPoint* getMap();
    //number of points in the map
    int getMapSize();

private:
	int32_t _num_buckets;
	int32_t _width;
	int32_t _height;
    MapIndex *_index;
    MapPoint *_map;
};

#endif