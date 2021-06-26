# beacon_gazebo_sim
## Gazebo plugins
### Receiver plugin
**source:** `src/receiver_model_plugin.cpp`

### Beacon plugin
**source:** `src/beacon_model_plugin.cpp`

### Sync plugin
**source:** `src/beacons_sync_world_plugin.cpp`



## Launch files
-  `robot_autonet_v1.launch`, `robot_autonet_v1.launch`:  spawns robot model and other things
-  `beacon.launch`: spawns beacon model
-  `test.launch`: test launch file



## Models
- `urdf/autonet_v1` : simple model \
  files:
    * `autonet_v1.xacro`
    * `autonet_v1_gazebo.xacro`
    * `macros.xacro`
- `urdf/vc_robot_v1` : simple model \
  files:
    * `vc_robot_v1.xacro`
    * `vc_robot_v1_gazebo.xacro`
    * `macros.xacro`
- `models/beacon` : simple beacon \
  files:
    * `model.sdf`
    * `model.config`
