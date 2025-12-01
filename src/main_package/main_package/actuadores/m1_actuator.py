#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import PWMOutputDevice, DigitalOutputDevice


class m1_actuator(Node):
    def __init__(self):
        super().__init__('m1_actuator_node')

        # -----------------------------
        #  PARAMETERS
        # -----------------------------
        self.declare_parameter('ain1')
        self.declare_parameter('ain2')
        self.declare_parameter('pwma')
        self.declare_parameter('pwm_frequency')
        self.declare_parameter('cmd_topic', 'motor_cmd')   # <--- NEW

        # Read parameters
        ain1 = self.get_parameter('ain1').value
        ain2 = self.get_parameter('ain2').value
        pwma = self.get_parameter('pwma').value
        pwm_freq = self.get_parameter('pwm_frequency').value
        cmd_topic = self.get_parameter('cmd_topic').value

        # Error check
        if not all([ain1, ain2, pwma, pwm_freq]):
            raise ValueError("Missing launch params: ain1, ain2, pwma, pwm_frequency")

        # -----------------------------
        #  HARDWARE SETUP
        # -----------------------------
        self.brake_on_zero = False
        self.pwm_max = 255
        self.current_cmd = 0.0

        self.dir1 = DigitalOutputDevice(ain1, active_high=True, initial_value=False)
        self.dir2 = DigitalOutputDevice(ain2, active_high=True, initial_value=False)
        self.pwm = PWMOutputDevice(pwma, frequency=pwm_freq, initial_value=0.0)

        # -----------------------------
        #  SUBSCRIBER TO COMMAND TOPIC
        # -----------------------------
        self.sub = self.create_subscription(
            Float32,
            cmd_topic,         # <--- topic is configurable now
            self.on_cmd,
            10
        )

        self.get_logger().info(
            f"m1_actuator: ready | AIN1={ain1} AIN2={ain2} PWMA={pwma} | fPWM={pwm_freq} Hz | cmd_topic={cmd_topic}"
        )

    # -----------------------------
    #  INTERNAL METHODS
    # -----------------------------
    def _set_direction(self, forward: bool):
        self.dir1.value = forward
        self.dir2.value = not forward

    def brake(self):
        self.dir1.on()
        self.dir2.on()
        self.pwm.value = 0.0

    def coast(self):
        self.dir1.off()
        self.dir2.off()
        self.pwm.value = 0.0

    def _apply_raw(self, cmd: float):
        if abs(cmd) < 0.01:
            if self.brake_on_zero:
                self.brake()
            else:
                self.coast()
            self.current_cmd = 0.0
            return

        try:
            forward = cmd >= 0
            self._set_direction(forward)
            self.pwm.value = abs(cmd)
            self.current_cmd = cmd

            self.get_logger().debug(
                f"m1_actuator: {'forward' if forward else 'reverse'} | PWM={abs(cmd):.2f}"
            )

        except Exception as e:
            self.get_logger().error(f"m1_actuator error: {e}")
            self.coast()

    # -----------------------------
    #  CALLBACK
    # -----------------------------
    def on_cmd(self, msg: Float32):
        cmd = max(-1.0, min(1.0, msg.data))
        self._apply_raw(cmd)

    # -----------------------------
    #  SHUTDOWN
    # -----------------------------
    def destroy_node(self):
        try:
            self.coast()
        finally:
            self.get_logger().info("m1_actuator shutdown.")
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = m1_actuator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("m1_actuator interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
