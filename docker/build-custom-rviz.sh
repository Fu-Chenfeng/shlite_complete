#!/usr/bin/env bash
set -euo pipefail
source /home/user/projects/shadow_robot/base/devel/setup.bash
workspace=/tmp/shlite-custom-rviz
mkdir -p "$workspace/src"
ln -sfn /project/rviz "$workspace/src/shlite_custom_rviz"
catkin_make -C "$workspace" -DCMAKE_BUILD_TYPE=Release -j2
python3 /project/app/custom_trajectory.py configure-rviz "$1"
