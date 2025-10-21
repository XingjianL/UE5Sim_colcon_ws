import os

input_folder = "//mnt/h/backups/windows_workstation/datasets/tpami_full_random_tomato/uncompressed_lower_seed"  # <-- Change this
output_script = "run_all.sh"
script_to_run = "RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_data_gen -- "   # <-- Replace with your actual script call

def parse_txt_file(filepath):
    args = []
    with open(filepath, 'r') as f:
        lines = [line.strip() for line in f if line.strip()]
    
    i = 0
    while i < len(lines):
        line = lines[i]
        if line.startswith("--"):
            flag = line
            values = []
            i += 1
            # Collect values until next --flag or EOF
            while i < len(lines) and not lines[i].startswith("--"):
                # Handle comma-separated on same line
                # if "," in lines[i]:
                #     values.extend(lines[i].split(","))
                # else:
                values.append(lines[i])
                i += 1
            # Append flag and its values (if any)
            args.append(flag)
            args.extend(values)
        else:
            i += 1
    return args

with open(output_script, "w") as sh:
    sh.write("#!/bin/bash\n\n")
    for fname in os.listdir(input_folder):
        if fname.endswith(".txt"):
            full_path = os.path.join(input_folder, fname)
            args = parse_txt_file(full_path)
            quoted_args = [f'"{a}"' if ' ' in a else a for a in args]
            command = f"{script_to_run} {' '.join(quoted_args)}"
            sh.write(command + "\n")

print(f"Generated shell script: {output_script}")
