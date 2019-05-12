#ifndef MAP_WRITER_H
#define MAP_WRITER_H

#include <stdio.h>
#include <math.h>
#include "MapIndex.h"
#include "MapPoint.h"

using namespace std;

class MapWriter
{

public:
    MapWriter(int32_t num_buckets, int32_t width, int32_t height);
    void addPoint(int16_t x, int16_t y);
    void dump(int count);
    int getIndexSizeBytes();
    int getMapSizeBytes();
    void getIndex(MapIndex *index);
    void getMap(MapPoint *map);
    int getNumBuckets();
    ~MapWriter();

private:
	int32_t _num_buckets;
	int32_t _width;
	int32_t _height;
	int32_t _bucket_size;
    MapIndex *_index;
    MapPoint *_map;
};

#endif