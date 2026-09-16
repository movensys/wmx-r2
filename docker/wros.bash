WMX_R2_CONTAINER="${WMX_R2_CONTAINER:-wmx_r2_container}"
WMX_R2_WORKSPACE="${WMX_R2_WORKSPACE:-/home/admin/workspaces/movensys_ws}"

wros() {
  local flags=(-i)
  [ -t 0 ] && flags=(-it)

  local setup='source /opt/ros/${ROS_DISTRO}/setup.bash'
  setup="${setup} && source ${WMX_R2_WORKSPACE}/install/setup.bash"

  if [ $# -eq 0 ]; then
    docker exec "${flags[@]}" -u root "${WMX_R2_CONTAINER}" \
      bash -lc "${setup} && exec bash -i"
  else
    docker exec "${flags[@]}" -u root "${WMX_R2_CONTAINER}" \
      bash -lc "${setup} && $*"
  fi
}
