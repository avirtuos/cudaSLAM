
all: main.cpp
	/usr/local/cuda-10.0/bin/nvcc -arch=sm_60 -o cudaSLAM -I . -I ./devel/ main.cpp MapReader.cu MapWriter.cpp CheckpointWriter.cpp toojpeg.cpp Map.cu ./lib/librplidar_sdk.a -rdc=true -lrt -lstdc++ -lpthread -lbluetooth