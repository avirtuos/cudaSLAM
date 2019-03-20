
all: main.cpp
	g++ -I . -I ./devel/  main.cpp ./lib/librplidar_sdk.a -lrt -lstdc++ -lpthread -fPIC