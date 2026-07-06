run_batch() {
    local base_temp=$1
    local temp_step=$2
    local intensity=$3
    local exposure=$4
    local base_seed=$5
    local offset=$6
    local rand_args=$7
    local preprocess=$8

    echo "=== Running base seed $base_seed with temp base $base_temp,$intensity,$exposure and offset y=$offset ==="

    local cmd="RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6_node -- \
--reset-time --light-temp \"$base_temp,$intensity,$exposure\" --seed \"$base_seed\" --split-height-leaf 300,16 \
--preprocess \"$preprocess\" --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand $rand_args --offset $offset --both"
    echo "$cmd"
    eval "$cmd"
    
    for i in {1..4}; do
        local temp=$((temp_step * i + base_temp))
        local seed=$((base_seed + i))
        local cmd="RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6_node -- \
--light-temp \"$temp,$intensity,$exposure\" --seed \"$seed\" --split-height-leaf 300,16 \
--preprocess \"$preprocess\" --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand $rand_args --offset $offset --both"
        echo "$cmd"
        eval "$cmd"
    done
}
# === Example usage: run_batch base_temp temp_step intensity exposure base_seed offset_y rand_args preprocess===

run_batch 9000 400 3 1 0 "0 -80 0 0 0 0" "10 10 10 0 0 0" "Lab"
# run_batch 9000 400 25 -3 10 "0 -80 0 0 0 0" "10 10 10 0 0 0" "Lab"
# run_batch 9000 400 50 -3 20 "0 -80 0 0 0 0" "10 10 10 0 0 0" "Lab"
