# Barrier Control Package

This package contains the core ROS2 nodes for implementing, simulating, and testing barrier function-based control laws on Kinova Gen3 robotic arms. It is designed to demonstrate advanced collision avoidance and provides utilities for easy testing and integration.

## Nodes

### `barrier_dynamics_controller.py`

This is the primary controller node that implements the safety-critical control law.

-   **Functionality**: It drives the robot to a target pose while guaranteeing that it remains within a safe set defined by one or more barrier functions. It uses a P-norm of the robot's state (joint positions and velocities) and a "descending ceiling" algorithm to handle dynamic obstacles and avoid instabilities when new barriers appear.
-   **Subscriptions**:
    -   `/target_pose` (`PoseStamped`): The desired end-effector pose.
    -   `/dynamic_joint_barriers` (`Float64MultiArray`): Receives real-time positions of dynamic obstacles, such as the joints of another robot.
-   **Publications**:
    -   `/joint_trajectory_controller/joint_trajectory` (`JointTrajectory`): Publishes low-latency joint position commands to the underlying `ros2_control` driver.
    -   `/arm_{index}/barrier_visuals` (`MarkerArray`): Publishes markers for visualizing static obstacles in RViz.
-   **Key Parameters**:
    -   `P`: A 12x12 diagonal matrix that weights the contribution of each joint's position and velocity to the state norm. **Tuning this is critical.**
    -   `lambda_1_0`, `lambda_2_0`, `r_1`, `r_2`: Parameters that define the shape and responsiveness of the adaptive barrier functions `λ1(x)` and `λ2(x)`.
    -   `barrier_gain`: A gain applied to the avoidance vector to control the strength of the repulsive force.

### `joint_barrier_publisher.py`

This node provides self-collision avoidance capabilities.

-   **Functionality**: It subscribes to the robot's `/joint_states`, uses `roboticstoolbox.fkine_all()` to efficiently calculate the Cartesian position of every link, and then publishes these positions as a series of spherical obstacles. The `barrier_dynamics_controller` can then subscribe to this topic to avoid collisions with itself or other robots.
-   **Publications**:
    -   `/dynamic_joint_barriers_markers` (`MarkerArray`): An array of sphere markers, each representing a collision body for a robot joint.
    -   `/dynamic_joint_barriers` (`Float64MultiArray`): An array of the information for each sphere barrier [`x`, `y`, `z`, `radius`].
-   **Key Parameters**:
    -   `joint_names_for_barriers`: A list of the joint/link names to create barriers for.
    -   `joint_barrier_radii`: A corresponding list of radii (in meters) for each spherical barrier.

### `target_handler.py`

A utility node for orchestrating the dual-arm system.

-   **Functionality**: Subscribes to a generic coordinate topic and intelligently routes the target to either `robot1` or `robot2` based on a simple spatial rule (the Y-coordinate of the target). It also computes a desired end-effector orientation, pointing towards the target while keeping the tool "upright" relative to the base.
-   **Subscriptions**:
    -   `/target_coordinates` (`Float64MultiArray`): A simple [x, y, z] coordinate.
-   **Publications**:
    -   `/robot1/target_pose` (`PoseStamped`)
    -   `/robot2/target_pose` (`PoseStamped`)

### `sequence_publisher_node.py`

A convenient testing utility.

-   **Functionality**: Publishes a pre-defined sequence of 3D coordinates to the `/target_coordinates` topic at a fixed interval, allowing for repeatable demonstrations and tests of the entire system.

### `rigid_body_dynamics_controller.py`

An alternative controller for comparison.

-   **Functionality**: Implements a more traditional resolved-motion rate control law based on a damped pseudo-inverse of the robot's Jacobian. This can be enabled via the `use_rigid_body_dynamics_controller` launch argument.

### `gripper_tcp_node.py`

A basic utility for controlling a gripper via a direct TCP socket connection, intended for hardware setups where standard ROS drivers are not used for the end-effector.
