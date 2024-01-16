#!/bin/bash

g++ -c ukf.cpp
g++ -I ./../Eigen main.cpp ukf.o -o main
./main