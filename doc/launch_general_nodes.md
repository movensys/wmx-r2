# Launch WMX R2 General Nodes
```
wros ros2 launch wmx_r2_package wmx_r2_general_nodes.launch.py \
      use_sim_time:=false \
      'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/config/wmx_r2_general_nodes_config.yaml' \
      'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/config/wmx_parameters.xml'
```