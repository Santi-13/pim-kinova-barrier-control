# PIM Kinova Barrier Control

This ROS2 Humble project provides a comprehensive framework for implementing and simulating advanced motion control strategies on single and dual Kinova Gen3 robotic arms. The core of the project is the implementation of **barrier function-based controllers** for dynamic obstacle and self-collision avoidance, alongside traditional control methods for comparison.

The system is designed for a dual-arm setup where two robots can work cooperatively in a shared workspace, intelligently receiving tasks and avoiding both static and dynamic obstacles, including each other's limbs.

## Core Features

-   **Barrier Function Control**: Implements a `BarrierDynamicsController` that uses a state-norm based barrier function to guarantee safety and forward invariance in the presence of obstacles.
-   **Dynamic Self-Collision Avoidance**: A dedicated `JointBarrierPublisher` node wraps each joint of the robot in a spherical barrier, publishing their real-time positions to prevent self-collisions.
-   **Dual-Arm Coordination**: Includes a `TargetHandler` node that intelligently dispatches tasks to one of two robots based on the target's location, and a `dual_gen3_control_launch.py` file to manage the entire dual-robot simulation.
-   **Swappable Control Architectures**: The main launch file allows easily switching between the custom `BarrierDynamicsController` and a more traditional `RigidBodyDynamicsController` (Jacobian-based) for comparison and testing.
-   **Simulation Ready**: Integrated with Ignition Gazebo for high-fidelity simulation and RViz for visualization.

## System Architecture

The nodes in the `barrier_control` package work together to achieve safe and intelligent motion.

```
┌─────────────────────────┐        ┌──────────────────┐       ┌───────────────────────────┐
│ sequence_publisher_node │──────▶│  target_handler   │─────▶│   robot1/target_pose      │
└─────────────────────────┘        └──────────────────┘       └───────────────────────────┘
                                           │                 ┌───────────────────────────┐
                                           └───────────────▶│   robot2/target_pose      │
                                                             └───────────────────────────┘

┌─────────────────────────┐        ┌──────────────────────────────┐        ┌─────────────────────────────┐
│ robotX/joint_states     ├──────▶│  barrier_dynamics_controller  ├──────▶│   robotX/joint_trajectory   │
└─────────────────────────┘        │       (or rigid_body)        │        └─────────────────────────────┘
                                   └─▲────────────────────────────┘
                                     │
┌─────────────────────────┐          │
│ joint_barrier_publisher │──────────┘
└─────────────────────────┘
```

## Getting Started

### Prerequisites

-   ROS2 Humble Hawksbill
-   Python 3.10+
-   `colcon` build tools
-   Required Python packages: `roboticstoolbox-python`, `numpy`, `scipy`, `quaternion`

### Installation & Build

1.  **Clone the repository's** workspace wherever you want:
    ```bash
    git clone https://github.com/Santi-13/pim-kinova-barrier-control.git -b Humble_ROS
    ```

2.  **Build the workspace** using `colcon`:
    ```bash
    cd ~/pim-kinova-barrier-control
    colcon build --symlink-install
    ```

3.  **Source the workspace**:
    ```bash
    source install/setup.bash
    ```

## Usage

The primary entry point for running the dual-arm simulation is the `dual_gen3_control_launch.py` file.

### Launching the Dual-Arm Simulation

To launch the full simulation with Ignition Gazebo and RViz, using barrier_dynamics_controller (default):

```bash
ros2 launch barrier_control dual_gen3_control_launch.py use_fake_hardware:=true sim_ignition:=true 
```

To just launch it in RViz2 and to use rigid_body_dynamics_controller:

```bash
ros2 launch barrier_control dual_gen3_control_launch.py use_fake_hardware:=true use_rigid_body_dynamics_controller:=true 
```

### Key Launch Arguments

-   `sim_ignition`: (default: `false`) Set to `true` to run the simulation in Ignition Gazebo.
-   `use_rigid_body_dynamics_controller`: (default: `false`) If `true`, launches the Jacobian-based controller instead of the default barrier function controller.
-   `use_fake_hardware`: (default: `false`) Toggle whether to try to connect to a real robot or not.

### Launching the Dual-Arm Real Control

To launch the dual control for real robots, using barrier_dynamics_controller:

```bash
ros2 launch barrier_control dual_gen3_control_launch.py 
```

### Key Launch Arguments

-   `robot1_ip_real` \ `robot2_ip_real` : (default: `192.168.1.10` \ `192.168.1.10` ) IP's set to each robot.
-   `initial_pose_x_robot1`, `initial_pose_y_robot1`, `initial_pose_z_robot1`, `initial_pose_roll_robot1`, `initial_pose_pitch_robot1`, `initial_pose_yaw_robot1` : (default: `0.0`, `0.15`, `0.0`, `0.0`, `0.0`, `0.0` ) You can set the position and rotation of each robot based on the `world` frame.

### Running a Demo

1.  **Launch the simulation** as shown above.
2.  In a new, sourced terminal, **run the sequence publisher** to send a series of targets:
    ```bash
    ros2 run barrier_control sequence_publisher_node
    ```

You should see the robots move to the targets in RViz, avoiding the static red spheres and their own joints (visualized as semi-transparent spheres).
