#!/bin/bash

# Source ROS 2
source /opt/ros/humble/setup.bash

# ── Auto-link packages from project root into workspace src ───────────────
mkdir -p /madara_ws/src

for pkg in /madara_project/madara_*/; do
  if [ -f "$pkg/package.xml" ]; then
    ln -sfn "$pkg" "/madara_ws/src/$(basename $pkg)"
  fi
done

# Source workspace if already built
if [ -f /madara_ws/install/setup.bash ]; then
  source /madara_ws/install/setup.bash
fi

exec "$@"