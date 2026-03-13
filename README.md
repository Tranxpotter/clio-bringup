# clio_bringup

ROS 2 bringup package for Innowing Clio Guide Robot Project mapping, localization, and navigation workflows.

## Overview

`clio_bringup` orchestrates:
- FAST-LIO odometry/mapping
- Localization against a saved `.pcd` map
- Nav2 navigation with a 2D occupancy map

## Launch Files

- `mapping.launch.py`  
  Runs mapping pipeline (driver + FAST-LIO, optional bag recording).

- `localization.launch.py`  
  Runs localization-only flow (localizer + pose remapper, optional driver/FAST-LIO/bag).

- `navigation.launch.py`  
  Runs full navigation stack (FAST-LIO + localizer + Nav2 + robot bridge).

- `register_localization.launch.py`  
  Integrated startup for relocalization + Nav2 with conditional components.

## Prerequisites

TODO: Confirm prerequisites

- ROS 2 Humble
- Workspace built with `colcon`
- Dependencies installed:
  - `livox_ros_driver2`
  - FAST-LIO stack
  - `localizer`
  - `guide_robot_localization`
  - Nav2


## Quick Start

### 1) Mapping (live sensor)

```bash
ros2 launch clio_bringup mapping.launch.py
```

### 2) Localization (live sensor)

```bash
ros2 launch clio_bringup register_localization.launch.py \
  map_path:=[path to your map] \
  map_2d_path:=[path to your 2d map yaml]
```

### 3) Localization (rosbag)

```bash
ros2 launch clio_bringup register_localization.launch.py \
  driver:=false \
  map_path:=[path to your map] \
  map_2d_path:=[path to your 2d map yaml] \
  use_sim_time:=True
```

In another terminal - 
```bash
ros2 bag play [path to your rosbag] --clock
```

### 4) Navigation (live sensor)

```bash
ros2 launch clio_bringup navigation.launch.py \
  map_path:=[path to your map] \
  map_2d_path:=[path to your 2d map yaml]
```

## Launch Arguments
Below are the launch arguments declared in each launch file.

### `mapping.launch.py`

| Argument | Default | Description |
|---|---|---|
| `driver` | `True` | Start Livox MID360 driver (`livox_ros_driver2`). |
| `fastlio` | `True` | Start FAST-LIO mapping launch. |
| `save_path` | `maps/scans.pcd` | Target path for saved map file (currently not actively used by shutdown copy logic). |
| `record_bag` | `True` | Record rosbag during mapping session. |
| `bag_path` | `rosbags/mapping` | Output path for recorded rosbag. |

### `register_localization.launch.py`

| Argument | Default | Description |
|---|---|---|
| `driver` | `True` | Start Livox MID360 driver. |
| `fastlio` | `True` | Start FAST-LIO pipeline. |
| `static_odom` | `True` | Start static odom publisher used to provide a fixed odometry stream/frame bridge for localization flow. |
| `localizer` | `True` | Start localizer nodes (`localizer_node` + localizer RViz). |
| `remapper` | `True` | Start initial pose remapper (`/initialpose` to relocalize service). |
| `map_path` | `maps/scans.pcd` | 3D `.pcd` map path for relocalization service. |
| `map_2d_path` | `maps/map.yaml` | 2D Nav2 map yaml path (`.yaml` + referenced `.pgm`). |
| `use_bag` | `False` | Play rosbag inside this launch. |
| `bag_path` | `rosbags/mapping` | Rosbag path used when `use_bag:=True`. |
| `use_sim_time` | `False` | Enable ROS simulation clock (`/clock`). |

### `navigation.launch.py`

| Argument | Default | Description |
|---|---|---|
| `driver` | `True` | Start Livox MID360 driver. |
| `fastlio` | `True` | Start FAST-LIO pipeline. |
| `static_odom` | `True` | Start static odom publisher for localization/nav flow. |
| `localizer` | `True` | Start localizer nodes (`localizer_node` + localizer RViz). |
| `remapper` | `True` | Start initial pose remapper for relocalization trigger. |
| `map_path` | `maps/scans.pcd` | 3D `.pcd` map path used by remapper/localizer flow. |
| `map_2d_path` | `maps/map.yaml` | 2D Nav2 map yaml path used by Nav2 bringup. |
| `use_sim_time` | `False` | Enable ROS simulation clock (`/clock`). |

### Quick guidance

- In practice, `map_path` and `map_2d_path` are the most important values to override per environment.
- For rosbag-based runs, set `use_sim_time:=True` and make sure bag playback publishes `/clock`.
- Disable components selectively (e.g., `driver:=False`) when running with pre-recorded data.

## Related Repositories

Check out these repositories as well, they are also used in this project: 
- https://github.com/Tranxpotter/2d-map-tools
- https://github.com/Tranxpotter/pcd-map-preprocessing

## Maintainer

- Name: `<maintainer_name>`
- Email: `<maintainer_email>`


## Starting Hardware interfaces
ros2 launch tron_utils system_exp.launch.py robot_head_model:=true
ros2 run clio_hardware eyes_node
conda activate stservo && ros2 run stservo_ros stservo_node.py --ros-args -p device_port:=/dev/ttyCLIO -p max_servo_id:=10 -p publish_rate:=75.0

## Starting main launch
ros2 launch tour_manager tour_planner.launch.py