#!/usr/bin/env bash
# Download the official ros2_control sources at the versions pinned in upstream.repos.
# Usage: scripts/upstream.sh
set -euo pipefail

# Always run from the repository root
cd "$(dirname "$0")/.."

# Packages we build. Every other package gets a COLCON_IGNORE file so colcon skips it.
declare -A KEEP=(
  [ros2_control]="hardware_interface controller_interface controller_manager controller_manager_msgs joint_limits ros2_control_test_assets transmission_interface"
  [ros2_controllers]="joint_state_broadcaster joint_trajectory_controller diff_drive_controller"
  [ros2_control_cmake]="ros2_control_cmake"
)

# Read upstream.repos line by line.
# Each line looks like:  upstream/NAME: {type: git, url: URL, version: COMMIT}
grep -E '^ +upstream/' upstream.repos | while IFS= read -r line; do
  dir=$(echo "$line" | sed -E 's/^ *([^:]+):.*/\1/')
  url=$(echo "$line" | sed -E 's/.*url: *([^,}]+).*/\1/')
  commit=$(echo "$line" | sed -E 's/.*version: *([^,} ]+).*/\1/')
  name=$(basename "$dir")

  # Clone the repository the first time
  if [ ! -d "$dir" ]; then
    git clone -q "$url" "$dir"
  fi

  # Move to the pinned commit and throw away any local changes
  git -C "$dir" fetch -q origin
  git -C "$dir" checkout -qf "$commit"
  git -C "$dir" clean -qfd

  # Mark packages we do not need, so colcon does not build them
  for pkg in $(find "$dir" -name package.xml -not -path '*/.git/*'); do
    pkg_dir=$(dirname "$pkg")
    [[ " ${KEEP[$name]} " == *" $(basename "$pkg_dir") "* ]] || touch "$pkg_dir/COLCON_IGNORE"
  done

  # Hide the COLCON_IGNORE files from git status inside upstream/
  echo COLCON_IGNORE > "$dir/.git/info/exclude"

  echo "$name: ready at ${commit:0:7}"
done
