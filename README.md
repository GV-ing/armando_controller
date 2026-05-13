## Available Nodes and Control Modes

The `armando_controller` package offers three main modes to manage the robot:

### Predefined Pose Control
Allows sending the robot to specific joint positions saved in the `config/poses.yaml` file. It uses an **Action Client** to ensure smooth and synchronized movements.

* **Run:**
    ```bash
    ros2 run armando_controller armando_controller_node --ros-args -p pose:="pos0"
    ```
*(Replace `pos0` with the desired pose name or `home` to reset).*

### Complete Automation
Executes a timed, automated "Pick and Place" sequence. The robot moves between different poses and activates/deactivates Gazebo plugins to grab and release the cubes.

* **Run:**
    ```bash
    ros2 run armando_controller armando_pick_place_node
    ```
* **Manual Gripper Control:**
    You can force the attachment or detachment of the cubes (ID 0-3) via the following topics:
    ```bash
    # Attach cube A (ID 0)
    ros2 topic pub --once /gripper/attach_a std_msgs/msg/Empty "{}"

    # Detach cube A (ID 0)
    ros2 topic pub --once /gripper/detach_a std_msgs/msg/Empty "{}"
    ```
*Cube mapping: `a=ID 0`, `b=ID 1`, `c=ID 2`, `d=ID 3`.*

### Dynamic IK Control
This node uses analytical Inverse Kinematics (IK) to reach targets (ArUco Markers) detected by the camera.

* **Run:**
    ```bash
    ros2 launch armando_controller armando_ik.launch.py
    ```
* **Real-Time Target Change:**
    Change the ID of the tracked cube without restarting the node:
    ```bash
    ros2 param set /armando_ik_node target_id 2
    ```
*(Use the desired ID or `-1` to return the arm to the Home position).*

## 🔧 Useful Commands & Debugging

**Send the robot to the Home position (0,0,0,0)**

If you want to manually reset the arm pose via the Action Server:
* ```bash
    ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
        trajectory: {
            joint_names: ['j0', 'j1', 'j2', 'j3'],
            points: [{
                positions: [0.0, 0.0, 0.0, 0.0],
                velocities: [0.0, 0.0, 0.0, 0.0],
                time_from_start: {sec: 2, nanosec: 0}
            }]
        }
    }"

Alternatively, you can use the controller node with the preset parameter:

* ```bash
    ros2 run armando_controller armando_controller_node --ros-args -p pose:="home"
