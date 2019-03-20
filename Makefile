
all: main.cpp
	g++ -ggdb -I . -I ./devel/  main.cpp ./lib/librplidar_sdk.a -lrt -lstdc++ -lpthread -fPIC