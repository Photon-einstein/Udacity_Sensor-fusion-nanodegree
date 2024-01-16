#!/bin/bash

g++ -c kalman_filter.cpp tracking.cpp
g++ -I ./../Eigen main.cpp kalman_filter.o tracking.o -o main
./main