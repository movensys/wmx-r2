# WMX R2 Packages

**Note that this ROS2 Packages requires pre-installed WMX Linux.**

## Bashrc Configuration [~/.bashrc]
```
export ROS_DOMAIN_ID=70                         #use any number
export ROS_DISTRO=jazzy                         #tested {jazzy, humble}
export CPU_ARCH=amd64                           #tested {amd64, arm64}
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

## Set CycloneDDS buffer
```
sudo tee /etc/sysctl.d/99-network-buffers.conf << 'EOF'
net.core.rmem_max=67108864
net.core.rmem_default=67108864
net.core.wmem_max=67108864
net.core.wmem_default=67108864
EOF

sudo sysctl -p /etc/sysctl.d/99-network-buffers.conf
sysctl net.core.rmem_max net.core.rmem_default net.core.wmem_max net.core.wmem_default
```


## Docker setup
```
cd ~/workspaces/movensys_ws/src/wmx-r2/docker
docker compose -f general.yaml -f wmx_r2.${CPU_ARCH}.yaml down
docker compose -f general.yaml -f wmx_r2.${CPU_ARCH}.yaml build
docker compose -f general.yaml -f wmx_r2.${CPU_ARCH}.yaml up -d
```