#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

# Import ADS1115
import board
import busio
from adafruit_ads1x15.ads1115 import ADS1115, Mode
from adafruit_ads1x15.analog_in import AnalogIn


class DistanceSensorNode(Node):
    def __init__(self):
        super().__init__("distance_sensor_node")

        # Declare input channel parameter (0–3)
        self.declare_parameter("adc_channel", 0)
        ch = int(self.get_parameter("adc_channel").value)

        if ch not in [0, 1, 2, 3]:
            raise ValueError("adc_channel debe ser 0, 1, 2 o 3.")

        # --- ADS1115 initialization ---
        i2c = busio.I2C(board.SCL, board.SDA)
        self.ads = ADS1115(i2c)
        
        # HIGH SPEED MODE (860 SPS)
        self.ads.mode = Mode.CONTINUOUS
        self.ads.data_rate = 860

        # Select channel
        self.channel = AnalogIn(self.ads, getattr(ADS1115, f'P{ch}'))

        # Publisher
        self.pub_dist = self.create_publisher(Float32, "distanceSensor_topic", 10)

        # Timer at 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            f"ADS1115 DistanceSensor initialized on channel {ch}"
        )

    def timer_callback(self):
        # Voltage returned directly by AnalogIn
        volts = self.channel.voltage

        # Clamp to 0–2V max
        volts = max(0.0, min(2.0, volts))

        # Map to distance (0–2V → 0–2cm)
        distance_cm = volts * 1.0  # 1 V = 1 cm

        msg = Float32()
        msg.data = distance_cm
        self.pub_dist.publish(msg)

        self.get_logger().debug(f"ADS1115: {volts:.3f} V → {distance_cm:.3f} cm")

    def destroy_node(self):
        self.get_logger().info("ADS1115 DistanceSensor shutdown")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DistanceSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ADS1115 node stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
