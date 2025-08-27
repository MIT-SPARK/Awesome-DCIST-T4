#!/bin/bash
source ${ADT4_WS}/install/setup.bash
config_generation_root=${ADT4_WS}/src/awesome_dcist_t4/dcist_launch_system/config_generation
config_output_dir=$config_generation_root/../config
tmux_output_dir=$config_generation_root/../tmux/
${ADT4_ENV}/spark_env/bin/python3 $config_generation_root/generate_configs.py $config_generation_root $config_output_dir $tmux_output_dir -v
