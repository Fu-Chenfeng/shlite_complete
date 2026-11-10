#!/bin/sh
# Right-click this file in Ubuntu Files and choose Run as a Program.
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd) || exit 1
exec /usr/bin/python3 "$project_dir/robotctl" panel sim
