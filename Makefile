
all: main.cpp
	/usr/local/cuda-10.0/bin/nvcc -arch=sm_60 -o cudaSLAM -I . -I ./devel/ main.cpp MapReader.cpp MapWriter.cpp CheckpointWriter.cpp toojpeg.cpp Map.cu ./lib/librplidar_sdk.a -lrt -lstdc++ -lpthread -lbluetooth