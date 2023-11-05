#!/bin/bash

FREQUENCY=${1:-"2100000"}

for i in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  echo "userspace" | sudo tee $i;
done

cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_available_frequencies

for i in /sys/devices/system/cpu/cpu*/cpufreq/scaling_setspeed; do
  echo "$FREQUENCY" | sudo tee $i;
done

sudo sh -c "echo 0 > /sys/devices/system/cpu/cpufreq/boost"