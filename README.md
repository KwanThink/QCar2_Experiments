# QCar2 ROS2 Projects – Running Guide

This README explains how to run the six QCar2 ROS2 projects included in this repository set.

The guide focuses on the practical steps required to build, launch, visualize, and configure each project. It assumes that ROS2 and the required project dependencies are already installed on the QCar2.

## 1. Overview

| Project | Main purpose |
|---|---|
| `ros2` | Manual driving and SLAM-based map creation |
| `Maps` | Occupancy maps (`.yaml` + `.pgm`) used by the localization and controller projects |
| `ros2_qcar2_nmpc_Mk1` | NMPC trajectory tracking toward a goal selected in RViz2 |
| `ros2_qcar2_nmpc_Mk2` | NMPC tracking of a predefined reference trajectory |
| `ros2_qcar2_flmpc_Mk1` | FLMPC trajectory tracking toward a goal selected in RViz2 |
| `ros2_qcar2_flmpc_Mk2` | FLMPC tracking of a predefined reference trajectory |
| `ros2_qcar2_oampc_Mk1c` | Obstacle-aware MPC tracking of a predefined reference trajectory |

The main difference between Mk1 and Mk2 is how `2D Goal Pose` in RViz2 is used:

- **Mk1:** the selected 2D Goal Pose defines the final trajectory goal.
- **Mk2:** the predefined trajectory is configured in the YAML file; 2D Goal Pose is only used as a trigger.
- **OAMPC Mk1c:** like Mk2, 2D Goal Pose is only a trigger for the predefined trajectory.

For NMPC and FLMPC, an acados solver must be generated before building the ROS2 workspace.

The provided `Maps` folder should be placed at `/home/nvidia/Maps`.

---

## 2. ROS2 – Manual Mapping

The `ros2` project is mainly used to manually drive the QCar2 and create a map using Cartographer SLAM.

### 2.1 Build the workspace

```bash
cd ~/ros2
colcon build
source install/setup.bash
```

### 2.2 Start manual cartography

```bash
ros2 launch qcar2_nodes qcar2_manual_cartographer_launch.py
```

This launch file starts the QCar2 hardware, LiDAR, joystick control, TF, and Cartographer.

### 2.3 Open RViz2

Open another terminal:

```bash
cd ~/ros2
source install/setup.bash
rviz2
```

In RViz2:

1. Set **Fixed Frame** to `map`.
2. Add **TF**.
3. Add **Map** and select `/map`.
4. Add **LaserScan** and select `/scan`.

### 2.4 Drive the QCar2 manually

The joystick controls implemented in this project are:

- **LB:** hold to arm/enable the joystick.
- **RT:** throttle.
- **A:** change driving direction.
- **Left joystick, horizontal axis:** steering.

Drive the vehicle through the environment until the required area has been mapped.

### 2.5 Save the map

Open another sourced terminal and run:

```bash
ros2 run nav2_map_server map_saver_cli -f /home/nvidia/Maps/my_map
```

Replace `my_map` with the desired map name.

---

## 3. NMPC Mk1

Workspace:

```text
ros2_qcar2_nmpc_Mk1
```

Configuration file:

```text
src/qcar2_nodes/config/qcar2_nmpc.yaml
```

### 3.1 Parameters to configure

The main trajectory parameters are:

| Parameter | Description |
|---|---|
| `number_of_waypoints` | Total number of waypoints used to generate the trajectory from the current position to the RViz2 goal |
| `segment_times` | Traveling time assigned to each trajectory segment |
| `result_directory` | Directory where experiment results are stored |

The number of values in `segment_times` must be:

```text
number_of_waypoints - 1
```

For example:

```yaml
number_of_waypoints: 4
segment_times: [2.0, 2.0, 2.0]
```

`trajectory_generation_mode` should remain set to `auto`.

### 3.2 Generate the acados solver

Before building the workspace:

```bash
cd ~/ros2_qcar2_nmpc_Mk1/src/qcar2_nodes/acados_solver
python generating_qcar2_solver.py
```

Then return to the workspace root:

```bash
cd ~/ros2_qcar2_nmpc_Mk1
```

### 3.3 Build and source

Before building, make sure the project is configured to use the correct map from `/home/nvidia/Maps` for the current environment.

```bash
colcon build
source install/setup.bash
```

### 3.4 Launch NMPC

```bash
ros2 launch qcar2_nodes qcar2_nmpc_bringup_launch.py
```

The default map is defined in the launch file. A different map can be selected directly from the command line:

```bash
ros2 launch qcar2_nodes qcar2_nmpc_bringup_launch.py map:=/home/nvidia/Maps/my_map.yaml
```

### 3.5 Open RViz2

Open another terminal:

```bash
cd ~/ros2_qcar2_nmpc_Mk1
source install/setup.bash
rviz2
```

Set **Fixed Frame** to `map`, then add:

- **TF**
- **Map** → `/map`
- **LaserScan** → `/scan`
- **Path** → `/qcar2_nmpc/reference_path`

### 3.6 Start an experiment

1. Use **2D Pose Estimate** to initialize the QCar2 pose on the map.
2. Use **2D Goal Pose** to select the desired final position and orientation.
3. NMPC automatically generates a trajectory from the current vehicle pose to the selected goal and starts tracking it.

---

## 4. NMPC Mk2

Workspace:

```text
ros2_qcar2_nmpc_Mk2
```

Configuration file:

```text
src/qcar2_nodes/config/qcar2_nmpc.yaml
```

### 4.1 Parameters to configure

Mk2 uses a predefined reference trajectory. The most important parameters are:

| Parameter | Description |
|---|---|
| `waypoints_ref_xy` | Reference trajectory waypoints in the form `[x0, y0, x1, y1, ...]` |
| `segment_times_ref` | Traveling time for each segment of the predefined reference trajectory |
| `number_of_waypoints_start` | Number of waypoints used for the automatically generated connection from the current vehicle pose `S0` to the first reference point `P0` |
| `segment_times_start` | Segment times for the connection from `S0` to `P0` |
| `theta_start` | Desired heading at the first reference point `P0` |
| `theta_end` | Desired heading at the final reference point `PN` |
| `result_directory` | Directory where experiment results are stored |

The predefined trajectory is therefore:

```text
S0 -> automatic start trajectory -> P0 -> P1 -> ... -> PN
```

For the fixed reference trajectory:

```text
number of segment_times_ref values = number of reference waypoints - 1
```

`shape_type` is used only to label the result folder. It does **not** automatically generate a trajectory shape. The actual trajectory is defined by `waypoints_ref_xy`.

### 4.2 Generate the acados solver

```bash
cd ~/ros2_qcar2_nmpc_Mk2/src/qcar2_nodes/acados_solver
python generating_qcar2_solver.py
cd ~/ros2_qcar2_nmpc_Mk2
```

### 4.3 Build and source

Before building, make sure the project is configured to use the correct map from `/home/nvidia/Maps` for the current environment.

```bash
colcon build
source install/setup.bash
```

### 4.4 Launch NMPC Mk2

```bash
ros2 launch qcar2_nodes qcar2_nmpc_bringup_launch.py
```

To use another map:

```bash
ros2 launch qcar2_nodes qcar2_nmpc_bringup_launch.py map:=/home/nvidia/Maps/my_map.yaml
```

### 4.5 Open RViz2

Open another terminal:

```bash
cd ~/ros2_qcar2_nmpc_Mk2
source install/setup.bash
rviz2
```

Set **Fixed Frame** to `map`, then add:

- **TF**
- **Map** → `/map`
- **LaserScan** → `/scan`
- **Path** → `/qcar2_nmpc/reference_path`

Only the reference path is required. There is no need to add `/qcar2_nmpc/active_path` in RViz2.

### 4.6 Start an experiment

1. Use **2D Pose Estimate** to initialize the vehicle pose.
2. Check the predefined reference path in RViz2.
3. Use **2D Goal Pose** to trigger the experiment.

In Mk2, the position and orientation selected with **2D Goal Pose are not used as the trajectory goal**. The predefined trajectory from `qcar2_nmpc.yaml` is used instead.

---

## 5. FLMPC Mk1

Workspace:

```text
ros2_qcar2_flmpc_Mk1
```

Configuration file:

```text
src/qcar2_nodes/config/qcar2_flmpc.yaml
```

### 5.1 Parameters to configure

| Parameter | Description |
|---|---|
| `number_of_waypoints` | Total number of waypoints used to generate the trajectory from the current position to the RViz2 goal |
| `segment_times` | Traveling time assigned to each trajectory segment |
| `result_directory` | Directory where experiment results are stored |

The number of values in `segment_times` must be `number_of_waypoints - 1`.

Example:

```yaml
number_of_waypoints: 4
segment_times: [3.0, 3.0, 3.0]
```

`trajectory_generation_mode` should remain set to `auto`.

### 5.2 Generate the acados solver

```bash
cd ~/ros2_qcar2_flmpc_Mk1/src/qcar2_nodes/acados_solver
python generating_qcar2_flmpc_solver.py
cd ~/ros2_qcar2_flmpc_Mk1
```

### 5.3 Build and source

Before building, make sure the project is configured to use the correct map from `/home/nvidia/Maps` for the current environment.

```bash
colcon build
source install/setup.bash
```

### 5.4 Launch FLMPC

```bash
ros2 launch qcar2_nodes qcar2_flmpc_bringup_launch.py
```

To use another map:

```bash
ros2 launch qcar2_nodes qcar2_flmpc_bringup_launch.py map:=/home/nvidia/Maps/my_map.yaml
```

### 5.5 Open RViz2

Open another terminal:

```bash
cd ~/ros2_qcar2_flmpc_Mk1
source install/setup.bash
rviz2
```

Set **Fixed Frame** to `map`, then add:

- **TF**
- **Map** → `/map`
- **LaserScan** → `/scan`
- **Path** → `/qcar2_flmpc/reference_path`

### 5.6 Start an experiment

1. Use **2D Pose Estimate** to initialize the QCar2 pose.
2. Use **2D Goal Pose** to select the desired final position and orientation.
3. FLMPC generates the trajectory to the selected goal and starts tracking it.

---

## 6. FLMPC Mk2

Workspace:

```text
ros2_qcar2_flmpc_Mk2
```

Configuration file:

```text
src/qcar2_nodes/config/qcar2_flmpc.yaml
```

### 6.1 Parameters to configure

| Parameter | Description |
|---|---|
| `waypoints_ref_xy` | Reference trajectory waypoints in the form `[x0, y0, x1, y1, ...]` |
| `segment_times_ref` | Traveling time for each segment of the predefined reference trajectory |
| `number_of_waypoints_start` | Number of waypoints used to connect the current vehicle pose `S0` to the first reference point `P0` |
| `segment_times_start` | Segment times for the connection from `S0` to `P0` |
| `theta_start` | Desired heading at `P0` |
| `theta_end` | Desired heading at `PN` |
| `result_directory` | Directory where experiment results are stored |

The trajectory structure is:

```text
S0 -> automatic start trajectory -> P0 -> P1 -> ... -> PN
```

The number of `segment_times_ref` values must equal the number of reference segments.

### 6.2 Generate the acados solver

```bash
cd ~/ros2_qcar2_flmpc_Mk2/src/qcar2_nodes/acados_solver
python generating_qcar2_flmpc_solver.py
cd ~/ros2_qcar2_flmpc_Mk2
```

### 6.3 Build and source

Before building, make sure the project is configured to use the correct map from `/home/nvidia/Maps` for the current environment.

```bash
colcon build
source install/setup.bash
```

### 6.4 Launch FLMPC Mk2

```bash
ros2 launch qcar2_nodes qcar2_flmpc_bringup_launch.py
```

To use another map:

```bash
ros2 launch qcar2_nodes qcar2_flmpc_bringup_launch.py map:=/home/nvidia/Maps/my_map.yaml
```

### 6.5 Open RViz2

Open another terminal:

```bash
cd ~/ros2_qcar2_flmpc_Mk2
source install/setup.bash
rviz2
```

Set **Fixed Frame** to `map`, then add:

- **TF**
- **Map** → `/map`
- **LaserScan** → `/scan`
- **Path** → `/qcar2_flmpc/reference_path`

Only the reference path is required. There is no need to add `/qcar2_flmpc/active_path` in RViz2.

### 6.6 Start an experiment

1. Use **2D Pose Estimate** to initialize the QCar2 pose.
2. Check the predefined reference path.
3. Use **2D Goal Pose** to trigger the experiment.

The position and orientation of the 2D Goal Pose are ignored; the trajectory configured in `qcar2_flmpc.yaml` is used.

---

## 7. OAMPC Mk1c

Workspace:

```text
ros2_qcar2_oampc_Mk1c
```

Configuration file:

```text
src/qcar2_nodes/config/qcar2_oampc.yaml
```

OAMPC Mk1c does **not** use acados, so there is no solver-generation step before `colcon build`.

### 7.1 Parameters to configure

The main trajectory parameters are:

| Parameter | Description |
|---|---|
| `waypoints_ref_xy` | Predefined reference trajectory |
| `segment_times_ref` | Traveling time for each reference segment |
| `number_of_waypoints_start` | Number of waypoints in the connection from `S0` to `P0` |
| `segment_times_start` | Segment times from `S0` to `P0` |
| `theta_start` | Desired heading at `P0` |
| `theta_end` | Desired heading at `PN` |
| `result_directory` | Directory where experiment results are stored |

The main obstacle-activation parameters are:

| Parameter | Description |
|---|---|
| `activation_radius` | Distance around the vehicle in which obstacles can become active |
| `activation_field_of_view` | Angular field of view used for obstacle activation |
| `max_active_obstacles` | Maximum number of simultaneously active obstacles |

Parameters such as `big_M`, `gamma`, `slack_weight`, `gurobi_time_limit`, `Q`, and `R` are controller-tuning parameters and normally do not need to be changed just to run the project.

### 7.2 Build and source

Before building, make sure the project is configured to use the correct map from `/home/nvidia/Maps` for the current environment.

```bash
cd ~/ros2_qcar2_oampc_Mk1c
colcon build
source install/setup.bash
```

### 7.3 Launch OAMPC

```bash
ros2 launch qcar2_nodes qcar2_oampc_bringup_launch.py
```

The default obstacle map input is selected by the launch file. A different occupancy map can be supplied with:

```bash
ros2 launch qcar2_nodes qcar2_oampc_bringup_launch.py map:=/home/nvidia/Maps/my_map.yaml
```

At launch time, the project automatically processes the selected occupancy map and generates:

```text
/home/nvidia/Maps/obstacle_map_oampc_Mk1c.yaml
```

The OAMPC controller starts only after this obstacle-map generation succeeds.

The `obstacle_map_file` parameter in `qcar2_oampc.yaml` points to the same generated file. If this output path is changed in the launch file, the YAML parameter must be changed accordingly.

### 7.4 Open RViz2

Open another terminal:

```bash
cd ~/ros2_qcar2_oampc_Mk1c
source install/setup.bash
rviz2
```

Set **Fixed Frame** to `map`, then add:

- **TF**
- **Map** → `/map`
- **LaserScan** → `/scan`
- **Path** → `/qcar2_oampc/reference_path`

The active path is not required for the basic RViz2 setup.

### 7.5 Start an experiment

1. Use **2D Pose Estimate** to initialize the QCar2 pose.
2. Check the predefined reference trajectory and obstacle map.
3. Use **2D Goal Pose** to trigger the experiment.

As with Mk2, the 2D Goal Pose is only a trigger. Its position and orientation do not define the trajectory.

---

## 8. When to regenerate the acados solver

For the NMPC and FLMPC projects, generate the solver at least once before the first `colcon build`.

You do **not** need to regenerate the solver when only changing trajectory parameters such as:

- `number_of_waypoints`
- `segment_times`
- `number_of_waypoints_start`
- `segment_times_start`
- `waypoints_ref_xy`
- `segment_times_ref`
- `theta_start`
- `theta_end`

Regenerate the solver before rebuilding if parameters used during solver generation are changed.

For **NMPC**, these include:

```text
wheelbase, Ts, mpc_N,
vx_min, vx_max,
delta_min, delta_max,
ax_min, ax_max,
Q, R, Qe
```

For **FLMPC**, these include:

```text
Ts, mpc_N,
delta_min, delta_max,
ax_min, ax_max,
Q, R
```

After regenerating the solver, return to the workspace root and run:

```bash
colcon build
source install/setup.bash
```
