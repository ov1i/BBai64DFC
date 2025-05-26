#!/bin/sh

LED_PATH="/sys/class/leds/beaglebone:green:usr1"

# Disable any system trigger
echo "none" > $LED_PATH/trigger

while true; do
    echo 1 > $LED_PATH/brightness
    sleep 1
    echo 0 > $LED_PATH/brightness
    sleep 1
done
