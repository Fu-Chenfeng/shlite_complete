#!/usr/bin/env bash
set -euo pipefail

project_dir=$(cd -- "$(dirname -- "$0")/.." && pwd)
source_file="$project_dir/docker/hand_object_collisions.cc"
build_dir=/tmp/shlite-contact-filter
library="$build_dir/libhand_object_collisions.so"
mkdir -p "$build_dir"
if [[ ! -f "$library" || "$source_file" -nt "$library" ]]; then
    # These development tools are already present in the bundled Shadow image.
    read -r -a compiler_flags <<< "$(pkg-config --cflags gazebo)"
    read -r -a linker_flags <<< "$(pkg-config --libs gazebo)"
    g++ -shared -fPIC "${compiler_flags[@]}" "$source_file" \
        -o "$library.tmp" "${linker_flags[@]}"
    mv "$library.tmp" "$library"
fi
