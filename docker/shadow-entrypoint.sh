#!/usr/bin/env bash
set -euo pipefail

# Keep project files read-only except trajectories. Prepare the image's own
# writable package files and remap its existing user to the host UID/GID.
target_uid=${LOCAL_USER_ID:-1000}
target_gid=${LOCAL_GROUP_ID:-1000}
if [[ "$(id -g user)" != "$target_gid" ]]; then
    groupmod -o -g "$target_gid" user
fi
if [[ "$(id -u user)" != "$target_uid" ]]; then
    usermod -o -u "$target_uid" user
fi
base=/home/user/projects/shadow_robot/base
multi="$base/src/sr_interface/sr_multi_moveit/sr_multi_moveit_config"
hand="$base/src/sr_common/sr_description/hand/config"
cp -a /project/config/hand/. "$hand/"
# The vendor simulation launch replaces GAZEBO_MODEL_PATH with this directory.
# Install the bundled models there too, so Gazebo needs no online model database.
models="$base/src/common_resources/sr_description_common/models"
mkdir -p "$models"
cp -a /project/assets/models/. "$models/"
cp /project/config/multi_robot_move_group_states.yaml "$multi/config/multi_robot_move_group_states.yaml"
mkdir -p /home/user/.ros /home/user/.gazebo /home/user/.config
for folder in /home/user/.ros /home/user/.gazebo /home/user/.config "$multi/config" "$multi/default_warehouse_mongo_db"; do
    if [[ -d "$folder" ]]; then chown -R user:user "$folder"; fi
done
export HOME=/home/user USER=user
exec /usr/local/bin/gosu user "$@"
