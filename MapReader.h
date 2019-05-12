#ifndef MAP_READER_H
#define MAP_READER_H

#include "MapIndex.h"
#include "MapPoint.h"

using namespace std;

class MapReader
{

public:
    MapReader(int32_t num_buckets, int32_t width, int32_t height, MapIndex *index, MapPoint *map);
    ~MapReader();
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