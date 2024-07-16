# Standard library imports

# Third-party imports
import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleOdometry
from numpy import NaN

# Pubsub node class definition
class FakeVisualOdometryPublisher(Node):

    # Pubsub constructor
    def __init__(self):
        super().__init__('fake_visual_odometry_publisher')  # Initialize node

        # Hardcoded PX4 topic
        px4_visual_odometry_topic = '/fmu/in/vehicle_visual_odometry'

        # Initialize publisher to PX4 vehicle_visual_odometry topic
        self.px4_publisher = self.create_publisher(VehicleOdometry, 
            px4_visual_odometry_topic, 10)

        # Initialize subscriber count
        self.prev_subscription_count = self.px4_publisher.get_subscription_count()
        self.log_subscription_state()

        # Create a timer to check for subscription changes
        self.subscription_check_timer = self.create_timer(1.0, self.check_subscriptions)

        # Create a timer to publish fake data
        self.fake_data_timer = self.create_timer(0.1, self.publish_fake_data)  # Publish fake data at 10Hz

        self.get_logger().info(f"PX4 fake visual odometry publisher node initialized with Publisher Topic: {px4_visual_odometry_topic}")

    # Method to publish fake data
    def publish_fake_data(self):
        msg_px4 = VehicleOdometry()  # Message to be sent to PX4

        # Set fixed position and orientation data
        msg_px4.pose_frame = 2  # FRD from px4 message
        msg_px4.position = [0.0, 0.0, 0.0]
        msg_px4.q = [1.0, 0.0, 0.0, 0.0]

        # Velocity components (unknown)
        msg_px4.velocity_frame = 2  # FRD from px4 message
        msg_px4.velocity = [NaN, NaN, NaN]
        msg_px4.angular_velocity = [NaN, NaN, NaN]

        # Variances
        msg_px4.position_variance = [0.0, 0.0, 0.0]
        msg_px4.orientation_variance = [0.0, 0.0, 0.0]
        msg_px4.velocity_variance = [0.0, 0.0, 0.0]

        self.px4_publisher.publish(msg_px4)  # Publish to PX4

    # Log the initial subscription state
    def log_subscription_state(self):
        if self.prev_subscription_count > 0:
            self.get_logger().info("/fmu/in/vehicle_visual_odometry is initially subscribed by a PX4 vehicle.")
        else:
            self.get_logger().warn("No subscribers initially on /fmu/in/vehicle_visual_odometry topic. Please check PX4 connection on ROS2 network.")

    # Check for subscription changes
    def check_subscriptions(self):
        current_subscription_count = self.px4_publisher.get_subscription_count()
        if current_subscription_count != self.prev_subscription_count:
            if current_subscription_count > 0:
                self.get_logger().info("/fmu/in/vehicle_visual_odometry is now subscribed by a PX4 vehicle.")
            else:
                self.get_logger().warn("No subscribers on /fmu/in/vehicle_visual_odometry topic. Please check PX4 connection on ROS2 network.")
            self.prev_subscription_count = current_subscription_count

def main(args=None):
    rclpy.init(args=args)

    fake_visual_odometry_publisher = FakeVisualOdometryPublisher()

    try:
        rclpy.spin(fake_visual_odometry_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        fake_visual_odometry_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
