import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from mpu6050 import mpu6050
import json
import time


class mpu6050(Node):
    def __init__(self):
        super().__init__("mpu6050_node")
        # --- Sensor initialization ---
        self.sensor = mpu6050(0x68)

        # --- Publishers ---
        self.pub_raw = self.create_publisher(String, "mpu6050_raw", 10)
        self.pub_dist = self.create_publisher(Float32, "mpu6050_distance_z", 10)

        # --- Integration variables ---
        self.vel_z = 0.0
        self.dist_z = 0.0
        self.last_time = time.time()

        # --- Exponential filter ---
        self.alpha = 0.1
        self.accel_z_filt = 0.0

        # --- Gravity calibration ---
        self.get_logger().info("Calibrating gravity offset (keep sensor still)...")
        samples = [self.sensor.get_accel_data()["z"] for _ in range(100)]
        self.g_offset = sum(samples) / len(samples)
        self.get_logger().info(f"Gravity offset calibrated: {self.g_offset:.3f} m/s²")

        # --- Timer setup (10 Hz) ---
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Ready(MPU6050_node).")

    # --- Read raw sensor data ---
    def _read_sensor(self):
        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()
        return accel, gyro

    # --- Integrate Z-axis distance ---
    def _integrate_distance(self, a_z_corr, dt):
        self.vel_z += a_z_corr * dt
        self.dist_z += self.vel_z * dt

    # --- Main timer callback ---
    def timer_callback(self):
        accel, gyro = self._read_sensor()
        a_z_raw = accel["z"]

        # Filter + gravity compensation
        a_z_filt = self._filter_accel(a_z_raw)
        a_z_corr = a_z_filt - self.g_offset

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        self._integrate_distance(a_z_corr, dt)

        # --- Publish raw data ---
        raw_msg = String()
        raw_msg.data = json.dumps({"accel": accel, "gyro": gyro})
        self.pub_raw.publish(raw_msg)

        # --- Publish Z distance ---
        dist_msg = Float32()
        dist_msg.data = float(self.dist_z)
        self.pub_dist.publish(dist_msg)

        self.get_logger().info(
            f"MPU6050_node: z={dist_msg.data:.4f} m | a={a_z_corr:.3f} m/s² (filtered)"
        )

    # --- Clean up ---
    def destroy_node(self):
        self.get_logger().info("Destroyed(MPU6050_node).")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped(MPU6050_node).")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

