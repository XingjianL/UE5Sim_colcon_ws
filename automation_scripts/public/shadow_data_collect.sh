#!/bin/bash

# Loop from 0 to 49
for seed in {0..0}
do
    echo "Running with seed $seed"
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 \
    ros2 run shadows shadow_data_collect -- --reduce-file-size --seed "$seed" --light-temp "10000,0,-1" \
    --ros-args --log-level shadow_camera:=debug
done