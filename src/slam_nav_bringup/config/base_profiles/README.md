# Base profiles

Base profiles override only chassis-specific parameters in the shared Nav2 configuration:

- MPPI motion model and sampled velocity bounds
- local and global costmap footprint
- velocity smoother limits, deadbands, and timeout
- safe command bridge limits, deadbands, and timeout

Built-in profiles:

```bash
base_profile:=omni
base_profile:=diff_drive
base_profile:=go2
```

They can be selected through the workspace entry points:

```bash
./run.sh nav-3d base_profile:=diff_drive
./run.sh nav-full base_profile:=go2
```

For a new chassis, copy the closest YAML file, update its footprint and measured motion limits,
then launch with an absolute path:

```bash
base_profile_file:=/path/to/my_robot.yaml
```

`base_profile_file` takes precedence over the named `base_profile`. A `DiffDrive` profile must set
all y-axis velocity and acceleration limits to zero. An `Omni` profile must provide positive y-axis
limits. The launch merge validates these rules before starting Nav2.

After changing a profile, rebuild the bringup package and verify the effective parameters:

```bash
./run.sh build
ros2 param get /controller_server FollowPath.motion_model
ros2 param get /controller_server FollowPath.vy_max
ros2 param get /safe_cmd_bridge_node max_vx
```
