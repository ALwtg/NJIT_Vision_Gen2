#!/bin/bash

cd ~/Desktop/NJIT_Vision/NJIT_Vision_Gen2

while true
do
    echo "Restarting program..."

    ./build/sentry

    echo "Program exited, restarting..."

     sleep 2
done
