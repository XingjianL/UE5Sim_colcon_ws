run_batch() {
    local base_temp=$1
    local temp_step=$2
    local intensity=$3
    local exposure=$4
    local base_seed=$5
    local offset=$6
    local rand_args=$7
    local preprocess=$8
    local split=$9
    local filter=${10}

    echo "=== Running base seed $base_seed with temp base $base_temp,$intensity,$exposure and offset y=$offset ==="

    local cmd="RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_instance_gen_node -- \
--reset-time --light-temp \"$base_temp,$intensity,$exposure\" --seed \"$base_seed\" --split-height-leaf $split \
--preprocess \"$preprocess\" --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand $rand_args --offset $offset \
--both --disease-filter $filter"
    echo "$cmd"
    eval "$cmd"
    
    for i in {1..1}; do
        local temp=$((temp_step * i + base_temp))
        local seed=$((base_seed + i))
        local cmd="RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_instance_gen_node -- \
--light-temp \"$temp,$intensity,$exposure\" --seed \"$seed\" --split-height-leaf $split \
--preprocess \"$preprocess\" --percent-healthy 0 --reduce-file-size --pcg-seed-incr 1 --rand $rand_args --offset $offset \
--both --disease-filter $filter"
        echo "$cmd"
        eval "$cmd"
    done
}
# === Example usage: run_batch base_temp temp_step intensity exposure base_seed offset_y rand_args preprocess===
FiltDisease="" # BS
BASE=1192026
run_batch 4000 1000 7000 -3 $((BASE+2000)) "200 -75 10 0 0 0" "0 0 0 0 0 0" "Lab" 400,0 "$FiltDisease"

# run_batch 4000 1000 7000 -3 $((BASE+2000)) "0 -75 10 0 0 0" "10 10 10 1 1 1" "Lab" 400,0 "$FiltDisease"
# run_batch 4000 1000 7000 -3.5 $((BASE+2100)) "0 -75 30 0 0 0" "10 10 10 1 1 1" "Lab" 400,0 "$FiltDisease"
# run_batch 4000 1000 7000 -4 $((BASE+2200)) "0 -75 50 0 0 0" "10 10 10 1 1 1" "Lab" 400,0 "$FiltDisease"
# run_batch 4000 1000 7000 -4.5 $((BASE+2300)) "0 -75 70 0 0 0" "10 10 10 1 1 1" "Lab" 400,0 "$FiltDisease"

# run_batch 4000 1000 7000 -3 $((BASE+3000)) "0 -75 10 0 0 0" "10 10 10 1 1 1" "Lab" 400,0 "$FiltDisease"
# run_batch 4000 1000 7000 -3.5 $((BASE+3100)) "0 -75 30 0 0 0" "10 10 10 1 1 1" "Lab" 400,0 "$FiltDisease"
# run_batch 4000 1000 7000 -4 $((BASE+3200)) "0 -75 50 0 0 0" "10 10 10 1 1 1" "Lab" 400,0 "$FiltDisease"
# run_batch 4000 1000 7000 -4.5 $((BASE+3300)) "0 -75 70 0 0 0" "10 10 10 1 1 1" "Lab" 400,0 "$FiltDisease"

# run_batch 4000 1000 7000 -3 $((BASE+4000)) "0 -75 10 0 0 0" "10 10 10 1 1 1" "Lab" 400,0 "$FiltDisease"
# run_batch 4000 1000 7000 -3.5 $((BASE+4100)) "0 -75 30 0 0 0" "10 10 10 1 1 1" "Lab" 400,0 "$FiltDisease"
# run_batch 4000 1000 7000 -4 $((BASE+4200)) "0 -75 50 0 0 0" "10 10 10 1 1 1" "Lab" 400,0 "$FiltDisease"
# run_batch 4000 1000 7000 -4.5 $((BASE+4300)) "0 -75 70 0 0 0" "10 10 10 1 1 1" "Lab" 400,0 "$FiltDisease"

# run_batch 4000 1000 7000 -3 $((BASE+5000)) "0 -75 10 0 0 0" "10 10 10 1 1 1" "Lab" 400,0 "$FiltDisease"
# run_batch 4000 1000 7000 -3.5 $((BASE+5100)) "0 -75 30 0 0 0" "10 10 10 1 1 1" "Lab" 400,0 "$FiltDisease"
# run_batch 4000 1000 7000 -4 $((BASE+5200)) "0 -75 50 0 0 0" "10 10 10 1 1 1" "Lab" 400,0 "$FiltDisease"
# run_batch 4000 1000 7000 -4.5 $((BASE+5300)) "0 -75 70 0 0 0" "10 10 10 1 1 1" "Lab" 400,0 "$FiltDisease"