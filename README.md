# F1Tenth-Reactive-Controller-_-Sepang
# F1TENTH Fast Follow-the-Gap Reactive Controller

## 1. Project Overview

This project is developed as part of the Autonomous Vehicles course.

The objective is to implement an individual reactive controller using the official F1TENTH simulator and ROS 2 Humble.

The controller is based on the Follow-the-Gap (FTG) algorithm, enhanced with a Disparity Extender, adaptive steering and speed control, and automatic speed tuning.

It processes LiDAR measurements to identify free driving space, avoid obstacles, and generate steering and velocity commands for autonomous navigation.

### Project Information

- Course: Autonomous Vehicles
- Simulator: F1TENTH Gym ROS
- Operating System: Ubuntu 22.04 LTS
- Middleware: ROS 2 Humble
- Programming Language: Python 3
- Navigation Algorithm: Follow-the-Gap
- Controller Version: v3
- Reference Track: Zandvoort

## 2. ROS 2 System Architecture

The controller uses a ROS 2 publish/subscribe architecture.

The implementation consists of two main components:

### 2.1 FastGapCore

The `FastGapCore` class implements the main reactive control algorithm.

Its responsibilities include:

- Processing LiDAR measurements.
- Detecting obstacle boundaries using the Disparity Extender.
- Identifying available free-space directions.
- Selecting a target steering direction.
- Calculating steering angles and vehicle speed.
- Tracking lap performance.
- Adjusting speed based on previous driving results.

### 2.2 FastGap ROS 2 Node

The `FastGap` class connects the algorithm to the F1TENTH simulator.

It receives sensor information, executes the controller, and publishes vehicle control commands.

### ROS 2 Communication Topics

| Topic | Direction | Function |
|---|---|---|
| `/scan` | Subscribe | Receives LiDAR measurements |
| `/ego_racecar/odom` | Subscribe | Receives vehicle position |
| `/drive` | Publish | Sends steering and speed commands |

### Message Types

- `/scan`: `sensor_msgs/msg/LaserScan`
- `/ego_racecar/odom`: `nav_msgs/msg/Odometry`
- `/drive`: `ackermann_msgs/msg/AckermannDriveStamped`

### System Workflow

1. The simulator publishes LiDAR and odometry data.
2. The ROS 2 node receives the sensor information.
3. The controller processes LiDAR measurements.
4. The Follow-the-Gap algorithm selects a target direction.
5. Steering and speed commands are calculated.
6. The controller publishes commands to `/drive`.

## 3. Mathematical Formulation and Algorithmic Approach

The controller processes each selected LiDAR scan through the following stages.

### Stage 1: LiDAR Preprocessing

The controller converts LiDAR measurements into a NumPy array.

Invalid measurements are handled using `numpy.nan_to_num`, and the distance values are clipped to the configured maximum range.

The forward field of view is limited to:

\[
-FOV \leq \theta \leq FOV
\]

Default parameters:

- Field of View: ±90°
- Maximum Range: 10.0 m

### Stage 2: Disparity Extender

The algorithm calculates the difference between adjacent LiDAR measurements.

If the difference exceeds the disparity threshold, the obstacle region is extended according to the vehicle's half-width.

The extension angle is calculated as:

\[
\Delta\theta =
\arctan\left(\frac{w}{d}\right)
\]

Where:

- \(w\): Vehicle half-width.
- \(d\): Distance to the closer obstacle.

Default parameters:

- Disparity Threshold: 0.5 m
- Vehicle Half-Width: 0.30 m

This method helps prevent the vehicle from selecting unsafe gaps near obstacle edges.

### Stage 3: Spatial Smoothing

A moving-average filter is applied to the extended LiDAR measurements.

The smoothing window is calculated using:

\[
W=\max\left(3,\operatorname{int}
\left(\frac{0.022}{\Delta\theta_{\text{LiDAR}}}\right)\right)
\]

This reduces sudden variations in the processed scan.

### Stage 4: Gap Detection and Target Selection

The controller identifies free-space directions using the processed LiDAR measurements.

A threshold is calculated from the maximum smoothed range:

\[
T=0.9\max(r_{\text{smooth}})
\]

Candidate directions are selected when:

\[
r_{\text{smooth}}[i]\geq T
\]

Among these candidates, the controller selects the direction closest to the vehicle's forward direction.

This approach favors open spaces while reducing unnecessary steering movements.

### Stage 5: Steering Control

The steering angle is calculated using proportional control:

\[
\delta_{\text{raw}}=K_p\theta_{\text{target}}
\]

The steering angle is limited to the maximum steering range.

A smoothing filter is then applied:

\[
\delta_t =
\alpha\delta_{\text{raw}}
+(1-\alpha)\delta_{t-1}
\]

Where:

- \(K_p\): Proportional gain.
- \(\alpha\): Steering smoothing factor.
- \(\delta_t\): Current steering angle.

Default parameters:

- \(K_p=1.0\)
- \(\alpha=0.8\)
- Maximum Steering Angle: 0.4189 rad

### Stage 6: Adaptive Speed Control

Vehicle speed is adjusted according to steering angle, front obstacle distance, and local speed limitations.

**1. Steering-based speed limitation**

\[
v_{\text{curve}} =
v_{\text{lim}}-
(v_{\text{lim}}-\min(v_{\min},v_{\text{lim}}))
\min\left(1,\frac{|\delta|}{s_{\text{ref}}}\right)
\]

**2. Front-obstacle braking**

\[
v_{\text{brake}} =
\sqrt{2a\max(d_{\text{front}}-d_{\text{margin}},0)}
\]

**3. Final speed selection**

\[
v=\max\left(
\min(v_{\min},v_{\text{lim}}),
\min(v_{\text{curve}},v_{\text{brake}},v_{\text{lim}})
\right)
\]

This allows the vehicle to reduce speed when approaching obstacles or making sharp turns.

### Stage 7: Automatic Speed Tuning and Lap Tracking

The controller monitors lap completion and evaluates the minimum measured side clearance.

After a clean lap:

- The successful maximum speed is recorded.
- The speed increases by the configured speed step.
- The controller continues testing higher speeds until reaching its configured limit or an unsuccessful speed boundary.

After a non-clean lap:

- The location associated with the minimum side clearance is identified.
- A local slow zone may be created or adjusted.
- The controller can return to a previously successful speed.

The controller also records lap times and performance information in a CSV file.

Default parameters:

- Initial Maximum Speed: 2.0 m/s
- Speed Increase Step: 0.25 m/s
- Maximum Speed Limit: 6.0 m/s
- Clean Clearance Threshold: 0.17 m

## 4. Code Structure

The source code is organized into two main classes and one main execution function.

### 4.1 FastGapCore

Responsible for the main Follow-the-Gap algorithm and vehicle control calculations.

| Function | Description |
|---|---|
| `__init__()` | Initializes controller parameters and states |
| `reset_stats()` | Resets lap clearance statistics |
| `side_clearance()` | Calculates lateral clearance from LiDAR data |
| `zone_factor()` | Determines local speed reduction |
| `lap_done()` | Evaluates completed laps and adjusts speed |
| `save()` | Saves controller state to a JSON file |
| `step()` | Executes the main reactive control algorithm |

### 4.2 FastGap

Responsible for ROS 2 communication and sensor callbacks.

| Function | Description |
|---|---|
| `__init__()` | Initializes the ROS 2 node, parameters, publishers, and subscribers |
| `odom_cb()` | Receives vehicle odometry |
| `scan_cb()` | Processes LiDAR scans and publishes driving commands |

### 4.3 Main Function

The `main()` function initializes ROS 2, creates the controller node, and starts the ROS 2 event loop using `rclpy.spin()`.

## 5. Code Explanation

This section explains how the main functions work and how the controller processes sensor information.

### 5.1 Importing Libraries and ROS 2 Messages

```python
import math
import json
import os
import numpy as np
```

- `math`: Performs mathematical calculations such as trigonometry and distance calculations.
- `json`: Saves and loads controller configuration and learned speed information.
- `os`: Checks whether state and log files exist.
- `numpy`: Processes LiDAR arrays and performs numerical operations.

The program also imports ROS 2 message types:

- `LaserScan`: LiDAR sensor measurements.
- `Odometry`: Vehicle position information.
- `AckermannDriveStamped`: Steering and speed commands.

The `try-except ImportError` block allows the core algorithm to be imported for testing even when ROS 2 is unavailable.

### 5.2 FastGapCore Initialization

```python
class FastGapCore:
    def __init__(self, p, log=print):
```

This class contains the main controller logic.

The constructor initializes:

- Maximum and minimum vehicle speeds.
- Braking parameters.
- Vehicle half-width.
- Disparity threshold.
- LiDAR field of view.
- Steering control parameters.
- Automatic speed tuning settings.
- Lap tracking variables.
- State and CSV file paths.

It also supports loading previously saved controller states through the `load` parameter.

### 5.3 Resetting Lap Statistics

```python
def reset_stats(self):
    self.min_clear = 99.0
    self.worst_xy = None
```

This function resets the minimum measured side clearance and the position associated with that measurement.

It is called when the controller starts a new lap evaluation.

### 5.4 Calculating Side Clearance

```python
def side_clearance(ranges, amin, inc):
```

This function estimates the lateral clearance between the vehicle and nearby walls.

The LiDAR measurements are converted into lateral distances using:

\[
d_{\text{lat}}=r|\sin(\theta)|
\]

Only LiDAR rays between 60° and 120° from the forward direction are considered.

The minimum valid lateral distance is returned as the side clearance.

### 5.5 Local Speed Reduction

```python
def zone_factor(self, x, y):
```

This function checks whether the vehicle is inside a previously identified slow zone.

The controller calculates the distance between the vehicle position and each stored slow zone.

If the vehicle is within 4 meters of a slow zone, the corresponding speed reduction factor is applied.

The smallest applicable factor is selected.

### 5.6 Lap Completion and Automatic Speed Tuning

```python
def lap_done(self, t):
```

This function is called when the controller detects a completed lap.

Its main operations are:

1. Calculate the lap time.
2. Update the total number of completed laps.
3. Record the lap time.
4. Evaluate whether the lap is CLEAN or DIRTY.
5. Print lap performance information.
6. Append the result to `laps.csv`.
7. Adjust the maximum speed when automatic tuning is enabled.
8. Save the updated controller state.

For a clean lap, the controller can increase its maximum speed.

For a non-clean lap, it may create a local slow zone or return to a previously successful speed.

### 5.7 Saving Controller State

```python
def save(self):
```

This function saves the controller's learned information into a JSON file.

The saved information includes:

- `good_vmax`: Highest successfully tested speed.
- `best_time`: Recorded best lap time associated with the saved successful speed.
- `bad_vmax`: Lowest recorded unsuccessful speed.
- `locked`: Whether speed tuning is locked.
- `zones`: Previously identified slow zones.

When freeze mode is enabled, the function does not save new states.

### 5.8 Main Reactive Control Function

```python
def step(self, ranges, amin, inc, x, y, t):
```

This is the most important function in the controller.

It processes sensor measurements and returns the steering angle and target vehicle speed.

The execution sequence is:

**Step A – Lap Detection**

The controller stores the initial vehicle position.

It detects a lap transition when the vehicle first moves more than 15 meters from the starting position and subsequently returns within 2 meters.

**Step B – Side Clearance Measurement**

The controller calculates side clearance and stores the minimum clearance observed during the lap.

**Step C – LiDAR Preprocessing**

The raw LiDAR data is converted into a NumPy array.

Invalid values are handled, and the range measurements are clipped to the configured maximum distance.

**Step D – Field-of-View Selection**

Only LiDAR rays within the configured forward field of view are processed for gap detection.

**Step E – Disparity Extender**

The controller calculates differences between adjacent LiDAR measurements.

When a disparity exceeds the configured threshold, the closer obstacle measurement is extended over neighboring rays according to the vehicle's half-width.

This reduces the possibility of selecting a narrow or unsafe opening.

**Step F – Spatial Smoothing**

The extended measurements are filtered using a moving-average convolution.

**Step G – Gap and Target Selection**

The controller identifies candidate directions using 90% of the maximum smoothed range.

It then selects the candidate closest to the forward direction.

**Step H – Steering Calculation**

The target angle is multiplied by the proportional gain.

The result is clipped to the maximum steering angle and smoothed using the previous steering command.

**Step I – Speed Calculation**

The controller calculates:

- Local speed limit.
- Steering-based speed.
- Front-obstacle braking speed.

The final speed is selected while respecting the configured minimum speed and local speed limit.

Finally, the function returns:

```python
return steer, float(v)
```

### 5.9 FastGap ROS 2 Node

```python
class FastGap(Node):
```

This class connects the core controller to the ROS 2 environment.

It creates:

- A publisher for `/drive`.
- A subscriber for `/ego_racecar/odom`.
- A subscriber for `/scan`.

It also declares ROS 2 parameters so the controller can be configured from the command line.

### 5.10 Odometry Callback

```python
def odom_cb(self, m):
    self.pos = (
        m.pose.pose.position.x,
        m.pose.pose.position.y
    )
```

This callback receives odometry messages and updates the vehicle's current position.

The position is later used for lap detection and local slow-zone calculations.

### 5.11 LiDAR Callback

```python
def scan_cb(self, msg):
```

This callback receives LiDAR measurements.

It performs the following operations:

1. Increments the LiDAR callback counter.
2. Checks whether vehicle position information is available.
3. Processes every fifth callback by default.
4. Obtains the current ROS 2 time.
5. Calls the `step()` function.
6. Creates an `AckermannDriveStamped` message.
7. Sets the steering angle and vehicle speed.
8. Publishes the command to `/drive`.

### 5.12 Main Function

```python
def main():
    rclpy.init()
    node = FastGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
```

The main function initializes the ROS 2 communication system, creates the controller node, and keeps it running until interrupted.

After execution stops, the node is destroyed and ROS 2 is shut down properly.

## 6. Installation and Configuration

### Prerequisites

- Ubuntu 22.04 LTS.
- ROS 2 Humble.
- Python 3.
- NumPy.
- Official F1TENTH Gym ROS simulator.

### Step 1: Install the F1TENTH Environment

Follow the official installation tutorial:

https://github.com/widegonz/F1Tenth-Repository

### Map Configuration

The F1TENTH Gym ROS simulator uses a map file to define the racing environment. By default, the configuration may use the `levine` map:

```yaml
map_path: '/home/F1Tenth-Repository/src/f1tenth_gym_ros/maps/levine'
```

In this configuration, the simulator loads the `levine.png` map.

To use a different map, follow these steps:

1. Place your map image (`.png`) in the `maps` directory:

   ```text
   F1Tenth-Repository/
   └── src/
       └── f1tenth_gym_ros/
           └── maps/
               ├── levine.png
               └── your_map.png
   ```

2. Open the simulator configuration file, usually `sim.yaml`.

3. Locate the `map_path` parameter and replace `levine` with the name of your map (without the `.png` extension):

   ```yaml
   map_path: '/home/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Sepang_map'
   ```

4. Save the configuration file and restart the simulator.

**Note:** Make sure the map image is placed in the correct directory and that the `map_path` matches the map filename. The corresponding map configuration files, such as the YAML map metadata, may also be required depending on the simulator setup.

Example configuration based on the reference setup:

```yaml
map_path: '/absolute/path/to/Sepang_map'
map_img_ext: '.png'
num_agent: 1
sx: 0.0
sy: 0.0
stheta: 1.22
```

The map path must point to the actual location of the map on the computer.

### Step 3: Install Dependencies

```bash
sudo apt install python3-numpy
```

Make sure the ROS 2 message packages required by the controller are available.

### Step 4: Launch the Simulator

Open the first terminal:

```bash
source ~/ros2_ws/install/setup.bash
cd ~/F1Tenth-Repository
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### Step 5: Run the Controller

Open another terminal:
### Controller Execution
Run the controller using the following command:
```bash
cd ~/Downloads
source install/setup.bash
python3 fast_gap_controller.py --ros-args \
-p vmax:=60.0 \
-p max_range:=60.0 \
-p decel:=7.0 \
-p autotune:=false \
-p kp:=0.7 \
-p alpha:=0.5 \
-p skip:=2 \
-p sref:=0.35
```

**Parameter Configuration**

| Parameter | Value | Description |
|---|---:|---|
| `vmax` | 60.0 | Maximum speed limit |
| `max_range` | 60.0 | Maximum LiDAR range used for preprocessing |
| `decel` | 7.0 | Deceleration parameter used in braking calculations |
| `autotune` | false | Disables automatic speed tuning |
| `kp` | 0.7 | Proportional steering gain |
| `alpha` | 0.5 | Steering smoothing factor |
| `skip` | 2 | Processes every second LiDAR callback |
| `sref` | 0.35 | Reference steering angle for speed adjustment |

**Note:** The actual vehicle speed is also constrained by obstacle distance, steering angle, and other controller limits. The `vmax` value is a configured upper limit, not a guarantee that the vehicle will reach that speed.
```

The controller starts with an initial maximum speed of 2.0 m/s.

## 7. Controller Parameters

| Parameter | Default | Description |
|---|---:|---|
| `vmax` | 2.0 | Initial maximum speed |
| `vmin` | 1.0 | Minimum speed |
| `decel` | 4.0 | Braking deceleration |
| `dmargin` | 0.8 | Braking distance margin |
| `half_width` | 0.30 | Vehicle half-width |
| `disp_thresh` | 0.5 | Disparity threshold |
| `max_range` | 10.0 | Maximum LiDAR range |
| `fov_deg` | 90.0 | Forward field of view on each side |
| `kp` | 1.0 | Steering proportional gain |
| `sref` | 0.25 | Steering reference |
| `alpha` | 0.8 | Steering smoothing factor |
| `vstep` | 0.25 | Speed increase per clean lap |
| `vmax_max` | 6.0 | Maximum allowed speed |
| `clear_thr` | 0.17 | Clean lap clearance threshold |
| `autotune` | true | Enables automatic speed tuning |
| `load` | false | Loads saved controller state |
| `freeze` | false | Disables tuning and state saving |
| `skip` | 5 | Processes every fifth LiDAR callback |

## 8. Lap Tracking and Performance Logging

The controller estimates lap completion using the vehicle's distance from its initial position.

A lap is detected after the vehicle:

1. Moves more than 15 meters from its starting position.
2. Returns within 2 meters of the starting position.

For every completed lap, the controller records:

- Lap number.
- Lap time.
- Current maximum speed.
- Minimum measured side clearance.
- Lap classification (CLEAN or DIRTY).

The results are stored in:

`laps.csv`

The learned speed and local slow-zone information are stored in:

`fast_gap_state.json`

## 9. Race Execution Mode

After completing the tuning phase, the controller can load its saved speed state and disable further automatic tuning.

Example:

```bash
python3 fast_gap_controller.py --ros-args \
-p vmax:=2.0 \
-p load:=true \
-p freeze:=true
```

This mode is intended for competition testing with a fixed learned speed configuration.

## 10. Testing and Performance Evaluation

The controller is evaluated according to the official course competition requirements.

### Challenge 1: 10 Consecutive Laps Without Collisions

The objective is to complete ten consecutive laps without colliding with the track walls.

### Challenge 2: Fastest Lap Time

The objective is to achieve the fastest valid lap time among the ten completed laps.

### Performance Evidence

The following evidence should be included:

- F1TENTH simulator screenshots.
- Controller execution logs.
- Ten-lap completion results.
- Fastest lap time.
- CSV performance records.

## 11. Conclusion

This project implements an enhanced Follow-the-Gap reactive controller for autonomous navigation in the F1TENTH simulation environment.

By combining LiDAR preprocessing, Disparity Extender, gap detection, steering control, adaptive speed control, and automatic speed tuning, the controller aims to balance driving safety and lap performance.

The implementation is evaluated through collision-free lap completion and fastest lap time in the assigned competition environment.

---

**Course:** Autonomous Vehicles  
**Project:** Individual Reactive Controller  
**Algorithm:** Fast Follow-the-Gap + Disparity Extender  
**Platform:** F1TENTH Gym ROS 2
