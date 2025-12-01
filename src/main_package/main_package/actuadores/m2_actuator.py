#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import DigitalOutputDevice


class m2_actuator(Node):
    def __init__(self):
        super().__init__('m2_actuator_node')

        self.declare_parameter("solenoid_pin")
        self.declare_parameter("cmd_topic", "m2_cmd")

        solenoid_pin = self.get_parameter("solenoid_pin").value
        cmd_topic = self.get_parameter("cmd_topic").value

        if solenoid_pin is None:
            raise ValueError("Missing parameter solenoid_pin")

        # Hardware
        self.solenoid = DigitalOutputDevice(solenoid_pin, active_high=True, initial_value=False)

        # Subscribe only to its corresponding controller
        self.sub = self.create_subscription(
            Float32,
            cmd_topic,
            self.on_cmd,
            10
        )

        self.get_logger().info(
            f"M2Actuator: solenoid_pin={solenoid_pin} listening to [{cmd_topic}]"
        )


    # -----------------------------------------------------------
    def on_cmd(self, msg: Float32):
        cmd = msg.data
        if cmd >= 0.5:
            self.solenoid.on()
            self.get_logger().debug("Solenoid → ON")
        else:
            self.solenoid.off()
            self.get_logger().debug("Solenoid → OFF")

    # -----------------------------------------------------------
    def destroy_node(self):
        try:
            self.solenoid.off()
        finally:
            self.get_logger().info("m2_actuator: shutdown (solenoid OFF).")
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = m2_actuator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("m2_actuator: keyboard interrupt.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
