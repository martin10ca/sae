#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn


class DistanceSensorNode(Node):
    def __init__(self):
        super().__init__("distance_sensor_node")

        # Declarar parámetro adc_channel (0–3)
        self.declare_parameter("adc_channel", 0)
        self.adc_channel = int(self.get_parameter("adc_channel").value)

        if self.adc_channel not in [0, 1, 2, 3]:
            raise ValueError("adc_channel must be 0–3")

        # Inicializar I2C + ADC
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.ads = ADS.ADS1115(self.i2c, address=0x48)
        self.ads.gain = 1  # ±4.096V

        # API moderna → canal es entero
        self.channel = AnalogIn(self.ads, self.adc_channel)

        # Publisher
        self.pub = self.create_publisher(Float32, "distanceSensor_topic", 10)

        # Timer 20Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(f"✔ ADS1115 iniciado en canal {self.adc_channel}")
        self.get_logger().info(f"Ready(distanceSensor_node in {self.adc_channel} channel + ADS1115).")

    def timer_callback(self):
        volts = self.channel.voltage  # Voltaje real del canal

        distance_cm = (volts / 3.3)  *2.0

        msg = Float32()
        msg.data = distance_cm
        self.pub.publish(msg)

        self.get_logger().debug(
            f"[ADS1115 CH={self.adc_channel}] {volts:.3f} V → {distance_cm:.3f} cm"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DistanceSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
