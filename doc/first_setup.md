# WMX R2 Packages

**Note that this ROS2 Packages requires pre-installed WMX Linux.**

## Bashrc Configuration [~/.bashrc]
```
export ROS_DOMAIN_ID=70                         #use any number
export ROS_DISTRO=jazzy                         #support {jazzy, humble}
export CPU_ARCH=amd64                           #support {amd64, arm64}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source ~/workspaces/movensys_ws/src/wmx-r2/docker/wros.bash
```
```
xhost +local:docker
source ~/.bashrc
```

## Git clone
```
mkdir -p ~/workspaces/movensys_ws/src
cd ~/workspaces/movensys_ws/src && \
   git clone https://github.com/movensys/wmx-r2.git
```

## Docker setup
```
cd ~/workspaces/movensys_ws/src/wmx-r2/docker
docker compose -f general.yaml -f wmx_r2.${CPU_ARCH}.yaml down
docker compose -f general.yaml -f wmx_r2.${CPU_ARCH}.yaml build
docker compose -f general.yaml -f wmx_r2.${CPU_ARCH}.yaml up -d
```