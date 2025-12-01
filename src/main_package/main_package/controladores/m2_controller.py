#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class m2_controller(Node):
    def __init__(self):
        super().__init__("m2_controller_node")

        # Parameters define the wiring
        self.declare_parameter("distance_topic", "distanceSensor_topic")
        self.declare_parameter("cmd_topic", "m2_cmd")

        self.distance_topic = self.get_parameter("distance_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value

        # Subscribe to selected distance sensor
        self.sub_dist = self.create_subscription(
            Float32,
            self.distance_topic,
            self.callback_distance,
            10
        )

        # Publish to the actuator assigned to this controller
        self.pub_cmd = self.create_publisher(Float32, self.cmd_topic, 10)

        self.get_logger().info(
            f"M2Controller: listening to [{self.distance_topic}] → publishing to [{self.cmd_topic}]"
        )

    def callback_distance(self, msg: Float32):
        distance = msg.data

        # Control logic
        threshold_cm = 1.0
        max_cmd = 1.0

        if distance > threshold_cm:
            cmd = max_cmd
        else:
            cmd = 0.0

        out = Float32()
        out.data = cmd
        self.pub_cmd.publish(out)

        self.get_logger().debug(
            f"Distance={distance:.2f} → cmd={cmd:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = m2_controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("m2_controller interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
