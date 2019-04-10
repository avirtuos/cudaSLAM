
all: main.cpp
	/usr/local/cuda-10.0/bin/nvcc -I . -I ./devel/ main.cpp  toojpeg.cpp ./lib/librplidar_sdk.a -lrt -lstdc++ -lpthread