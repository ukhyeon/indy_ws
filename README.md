# Real-Time Human–Robot Collaboration: Indy7 Safety Control

This ROS 2 workspace implements the robot-side dynamics, collision-risk
assessment, and speed-control pipeline for an Indy7 collaborative robot. It
combines robot state and human dynamics information to estimate potential
collision severity and generate a risk-aware speed-scale command.

The human sensing, pose estimation, human dynamics, and shared ROS 2 message
definitions are maintained in [`hrc_ws`](https://github.com/ukhyeon/hrc_ws).

## System overview

```text
Human state and effective mass (hrc_ws) ─┐
                                        ├──► Collision-risk assessment
Indy7 state and robot effective mass ────┘              │
                                                        ▼
                                              Speed-scale command
                                                        │
                                                        ▼
                                                     Indy7
```

The robot-side pipeline performs the following tasks:

1. Read the Indy7 state through the robot communication interface.
2. Compute robot dynamics and directional effective mass using Pinocchio.
3. Combine human and robot states to assess candidate collisions.
4. Apply ISO/TS 15066-related force limits during risk assessment.
5. Publish and apply a smoothed robot speed-scale command.

## Package

### `indy_control_cpp`

C++ ROS 2 package containing the production nodes and diagnostic utilities.

| Executable | Role |
| --- | --- |
| `robot_dynamics_node` | Publishes robot dynamics and answers effective-mass queries |
| `collision_assessment_node` | Evaluates collision candidates and generates speed commands |
| `indy_speed_control_node` | Sends speed-scale commands to the Indy7 controller |
| `indy_connection_check_node` | Checks communication with the robot |
| `indy_speed_check_node` | Validates robot state and speed behavior |
| `indy_pinocchio_meff_check` | Checks Pinocchio-based effective-mass calculations |
| `indy_pinocchio_meff_with_gearing` | Supports effective-mass validation with gearing |
| `collision_assessment_check` | Runs collision-assessment checks without the full pipeline |

The package also contains the IndyDCP3 client implementation, generated gRPC
sources, the Indy7 URDF used for dynamics calculations, and ISO 15066-related
configuration data.

## Repository layout

```text
indy_ws/
├── src/
│   ├── indy_control_cpp/
│   │   ├── include/          # IndyDCP3 and performance-logging headers
│   │   ├── proto/            # Generated Protobuf/gRPC sources
│   │   ├── ISO15066/         # Collision-force configuration
│   │   └── src/              # ROS 2 nodes and checks
│   └── urdf_file/            # Indy7 robot model
└── README.md
```

Generated `build/`, `install/`, and `log/` directories are local ROS 2 build
artifacts and are not part of the source distribution.

## Prerequisites

- ROS 2 with `colcon`
- A C++17 compiler
- [`hrc_ws`](https://github.com/ukhyeon/hrc_ws), providing `hrc_interfaces`
- Pinocchio
- Eigen3
- nlohmann/json
- Protobuf and gRPC
- Network access to an Indy7 controller for hardware execution

## Build

Build and source `hrc_ws` first so that the shared message package is available:

```bash
source /opt/ros/<ros-distro>/setup.bash
cd /path/to/hrc_ws
colcon build --symlink-install
source install/setup.bash
```

Then build this workspace:

```bash
cd /path/to/indy_ws
colcon build --symlink-install
source install/setup.bash
```

Replace `<ros-distro>` with the ROS 2 distribution installed on the system.

## Run

With both workspaces sourced, the core robot-side nodes can be started in
separate terminals:

```bash
ros2 run indy_control_cpp robot_dynamics_node
```

```bash
ros2 run indy_control_cpp collision_assessment_node
```

```bash
ros2 run indy_control_cpp indy_speed_control_node
```

Start the required sensing and human-dynamics nodes from `hrc_ws` before running
the complete collision-aware control pipeline.

> [!CAUTION]
> The speed-control node communicates with real robot hardware. Verify the robot
> address, workspace clearance, emergency-stop operation, and controller mode
> before execution. Begin testing at a conservative speed in a controlled area.

## Configuration

The current implementation includes configuration for:

- robot controller address;
- Indy7 URDF path;
- ISO/TS 15066-related force thresholds;
- experiment and performance-log output paths;
- collision-assessment and speed-smoothing parameters.

Some values are currently defined in source code for the original development
environment. Review them before building or connecting to hardware.

## Current limitations

- The workspace depends on `hrc_interfaces` from the separate `hrc_ws` repository.
- Several file paths and the default robot address are specific to the original
  development environment.
- The URDF references mesh files from an external Indy ROS 2 installation.
- Hardware-dependent behavior cannot be reproduced without a compatible Indy7
  controller and an appropriately configured network.

## Related repository

- [`hrc_ws`](https://github.com/ukhyeon/hrc_ws) — human sensing, pose estimation,
  human dynamics, visualization, and shared ROS 2 interfaces

