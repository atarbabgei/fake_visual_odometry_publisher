# fake_visual_odometry_publisher

`fake_visual_odometry_publisher` is a ROS2 package that publishes fake visual odometry data to the PX4 Autopilot. This package continuously sends fixed position and orientation data to the `fmu/out/vehicle_visual_odometry` topic.

## Usage

### Running the Node Directly

To run the `fake_visual_odometry_publisher` node directly, use the following command:


or via launch file
```bash
ros2 launch fake_visual_odometry_publisher start.launch.py
```