#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8MultiArray


class m1_controller(Node):
    def __init__(self):
        super().__init__('m1_controller_node')

        # controller parameters
        self.frequency_hz = 20.0
        self.ramp_step = 5
        self.pwm_max = 255

        # internal state
        self.max_pct = 0.0
        self.current_pwm = 0
        self.max_pwm = 0
        self.direction = 1

        # ----------- PUBLISHERS FOR TWO MOTORS -------------
        self.pub_m1 = self.create_publisher(Float32, '/m1/cmd', 10)
        self.pub_m2 = self.create_publisher(Float32, '/m2/cmd', 10)

        # ----------- SUBSCRIPTIONS --------------------------
        self.sub_pct = self.create_subscription(
            Float32, 'max_speed_pct', self.callback_max_speed, 10
        )

        self.sub_tcrt = self.create_subscription(
            Int8MultiArray, "tcrt5000_topic", self.callback_tcrt5000, 10
        )

        # CONTROL LOOP TIMER
        self.timer = self.create_timer(1.0 / self.frequency_hz, self.control_loop)

        self.get_logger().info(
            f"m1_controller: ready | freq={self.frequency_hz} Hz | ramp_step={self.ramp_step}"
        )

    # ------------------------------------------------------
    def callback_max_speed(self, msg):
        self.max_pct = max(-100.0, min(msg.data, 100.0))

        self.direction = 1 if self.max_pct >= 0 else -1
        self.max_pwm = round((abs(self.max_pct) / 100.0) * self.pwm_max)

        self.get_logger().info(f"New max speed: {self.max_pct:.1f}%")

    def callback_tcrt5000(self, msg):
        self.sensor_values = msg.data
        self.get_logger().debug(f"TCRT5000 readings: {self.sensor_values}")

    # ------------------------------------------------------
    def control_loop(self):

        # Ramp-up logic
        if self.current_pwm < self.max_pwm:
            self.current_pwm = min(self.current_pwm + self.ramp_step, self.max_pwm)
        elif self.current_pwm > self.max_pwm:
            self.current_pwm = max(self.current_pwm - self.ramp_step, self.max_pwm)

        normalized_cmd = (self.current_pwm / self.pwm_max) * self.direction

        msg = Float32()
        msg.data = normalized_cmd

        # -------- PUBLISH TO BOTH MOTORS SAFELY --------
        # ROS2 does NOT crash if no subscriber exists.
        try:
            self.pub_m1.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"m1_controller: M1 publish failed → {e}")

        try:
            self.pub_m2.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"m1_controller: M2 publish failed → {e}")

        self.get_logger().debug(
            f"PWM={self.current_pwm}/{self.pwm_max} dir={self.direction:+d} cmd={normalized_cmd:+.2f}"
        )

    # ------------------------------------------------------
    def destroy_node(self):
        stop_msg = Float32()
        stop_msg.data = 0.0

        # Attempt to stop motors, ignoring missing actuators
        try:
            self.pub_m1.publish(stop_msg)
        except:
            pass

        try:
            self.pub_m2.publish(stop_msg)
        except:
            pass

        self.get_logger().info("m1_controller: shutdown (motors stopped).")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = m1_controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("m1_controller: keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
