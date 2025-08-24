#!/bin/bash

sh_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

output_base_dir_name="rosbag2_out"
output_base_dir_path="${sh_dir}/${output_base_dir_name}"

mkdir -p "${output_base_dir_path}"

output_dir_path="${output_base_dir_path}/rosbag2_$(date +%Y-%m-%d_%H-%M-%S)/"
echo "rosbag2 output directory: ${output_dir_path}"

ros2 bag record -o ${output_dir_path} \
/gnc/pos_eci