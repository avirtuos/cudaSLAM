#!/bin/bash
ulimit -c unlimited
ulimit -a
rm -f ./map_*jpg
./a.out /dev/ttyUSB0 256000
