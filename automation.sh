# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,5,1 --both --seed 40508 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,4,1 --both --seed 40518 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,3,1 --both --seed 40528 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,2,1 --both --seed 40538 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,0,1 --both --seed 40548 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,0,1 --both --seed 40558 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,0,1 --both --seed 40568 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,0,1 --both --seed 40578 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,2,1 --both --seed 40588 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,3,1 --both --seed 40598 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1

#RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,0,1 --seed 400 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- --reset-time \
# --light-temp 10000,0,0 --seed 400 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 0 --rand 0 0 0 5 0 10 --offset 0 0 0 0 0 0
# for i in {1..5}; do
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- \
# --light-temp 10000,0,0 --seed "$i" --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand 0 0 0 5 0 10 --offset 0 0 0 0 0 0
# done

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- --reset-time \
# --light-temp 1000,0,-2 --seed 406 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand 0 0 0 5 0 10 --offset 0 10 0 0 0 0
# for i in {1..5}; do
# temp=$((400 * i + 1000))
# seed=$((i + 406))
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- \
# --light-temp "${temp},0,-2" --seed "$seed" --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand 0 0 0 5 0 10 --offset 0 10 0 0 0 0
# done

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- --reset-time \
# --light-temp 3000,0,0 --seed 4011 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand 0 0 0 5 0 10 --offset 0 20 0 0 0 0
# for i in {1..5}; do
# temp=$((400 * i + 3000))
# seed=$((i + 4011))
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- \
# --light-temp "${temp},0,0" --seed "$seed" --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand 0 0 0 5 0 10 --offset 0 20 0 0 0 0
# done

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- --reset-time \
# --light-temp 5000,0,0 --seed 4016 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand 0 0 0 5 0 10 --offset 0 30 0 0 0 0
# for i in {1..5}; do
# temp=$((400 * i + 5000))
# seed=$((i + 4016))
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- \
# --light-temp "${temp},0,-0.5" --seed "$seed" --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand 0 0 0 5 0 10 --offset 0 30 0 0 0 0
# done

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- --reset-time \
# --light-temp 7000,0,0 --seed 4021 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand 0 0 0 5 0 10 --offset 0 40 0 0 0 0
# for i in {1..5}; do
# temp=$((400 * i + 7000))
# seed=$((i + 4021))
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- \
# --light-temp "${temp},0,0" --seed "$seed" --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand 0 0 0 5 0 10 --offset 0 40 0 0 0 0
# done

# ### No lights
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- --reset-time \
# --light-temp 10000,0,1 --seed 4026 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand 0 0 0 5 0 10 --offset 0 50 0 0 0 0
# for i in {1..5}; do
# temp=$((400 * i + 7000))
# seed=$((i + 4026))
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- \
# --light-temp "${temp},0,1" --seed "$seed" --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand 0 0 0 5 0 10 --offset 0 50 0 0 0 0
# done

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- --reset-time \
# --light-temp 10000,0,-0.5 --seed 4031 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand 0 0 0 5 0 10 --offset 0 60 0 0 0 0
# for i in {1..5}; do
# temp=$((400 * i + 7000))
# seed=$((i + 4031))
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- \
# --light-temp "${temp},0,-0.5" --seed "$seed" --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand 0 0 0 5 0 10 --offset 0 60 0 0 0 0
# done

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- --reset-time \
# --light-temp 10000,0,-0.5 --seed 4036 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand 0 0 0 5 0 10 --offset 0 70 0 0 0 0
# for i in {1..5}; do
# temp=$((1000 * i + 3000))
# seed=$((i + 4036))
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- \
# --light-temp "${temp},0,-0.5" --seed "$seed" --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand 0 0 0 5 0 10 --offset 0 70 0 0 0 0
# done

#RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- --reset-time \
#--light-temp 10000,0,-1 --seed 4041 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand 0 0 0 5 0 10 --offset 0 80 0 0 0 0
#for i in {1..5}; do
#temp=$((400 * i + 7000))
#seed=$((i + 4041))
#RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- \
#--light-temp "${temp},0,-1" --seed "$seed" --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand 0 0 0 5 0 10 --offset 0 80 0 0 0 0
#done
#
#RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- --reset-time \
#--light-temp 10000,0,-1 --seed 4046 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand 0 0 0 5 0 10 --offset 0 90 0 0 0 0
# for i in {5..5}; do
# temp=$((1000 * i + 3000))
# seed=$((i + 4046))
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- \
# --light-temp "${temp},0,-1" --seed "$seed" --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand 0 0 0 5 0 10 --offset 0 90 0 0 0 0
# done


# kill -9 $(pgrep benchbot_xarm6_cpp)
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
# -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pred "0,0,10" --pcg-seed-incr 0 --ros-args --log-level error
# kill -9 $(pgrep benchbot_xarm6_cpp)
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
# -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 14 --pcg-seed-incr 0 

# kill -9 $(pgrep benchbot_xarm6_cpp)
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
# -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pred "0,0,10" --pcg-seed-incr 17
# kill -9 $(pgrep benchbot_xarm6_cpp)
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
# -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 32 --pcg-seed-incr 0 

# kill -9 $(pgrep benchbot_xarm6_cpp)
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
# -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pred "0,0,10" --pcg-seed-incr 2
# kill -9 $(pgrep benchbot_xarm6_cpp)
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
# -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 34 --pcg-seed-incr 0 