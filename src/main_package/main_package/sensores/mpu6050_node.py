#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from mpu6050 import mpu6050
import json


class MPU6050Node(Node):
    def __init__(self):
        super().__init__("mpu6050_node")

        # Init sensor
        self.sensor = mpu6050(0x68)

        # Publisher (solo datos crudos)
        self.pub_raw = self.create_publisher(String, "mpu6050_raw", 10)
        self.pub_accel_z = self.create_publisher(Float32, "mpu6050_accel_z", 10)

        # Timer 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Ready(MPU6050_node).")

    def timer_callback(self):
        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()

        msg = String()
        msg.data = json.dumps({"accel": accel, "gyro": gyro})
        self.pub_raw.publish(msg)

        accel_z_msg = Float32()
        accel_z_msg.data = accel["z"]
        self.pub_accel_z.publish(accel_z_msg)


    def destroy_node(self):
        self.get_logger().info("Destroyed(MPU6050_node).")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
