import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from time import sleep


class m1_actuator(Node):
    def __init__(self):
        super().__init__('m1_actuator_node')

        # Declare param
        self.declare_parameter('ain1')
        self.declare_parameter('ain2')
        self.declare_parameter('pwma')
        self.declare_parameter('pwm_frequency')

        # Verify all param arrived
        ain1 = self.get_parameter('ain1').get_parameter_value().integer_value
        ain2 = self.get_parameter('ain2').get_parameter_value().integer_value
        pwma = self.get_parameter('pwma').get_parameter_value().integer_value
        pwm_freq = self.get_parameter('pwm_frequency').get_parameter_value().integer_value

        # If any param missing, raise error
        if not all([ain1, ain2, pwma, pwm_freq]):
            raise ValueError("param missing in launch file: ain1, ain2, pwma, pwm_frequency.")

        # aditional config
        self.brake_on_zero = False
        self.ramp_ms = 120
        self.step_us = 2000
        self.pwm_max = 255
        self.current_cmd = 0.0

        # init hardware
        self.dir1 = DigitalOutputDevice(ain1, active_high=True, initial_value=False)
        self.dir2 = DigitalOutputDevice(ain2, active_high=True, initial_value=False)
        self.pwm = PWMOutputDevice(pwma, frequency=pwm_freq, initial_value=0.0)
        
        # create suscription -> on_cmd method
        self.sub = self.create_subscription(Float32, 'motor_cmd', self.on_cmd, 10)

        self.get_logger().info(
            f"m1_actuator: ready | AIN1={ain1} AIN2={ain2} PWMA={pwma} | fPWM={pwm_freq} Hz"
        )

    # Methods
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
            pwm_value = abs(cmd)
            self.pwm.value = pwm_value
            self.current_cmd = cmd

            self.get_logger().debug(
                f"m1_actuator: {'forward' if forward else 'reverse'} | PWM={pwm_value:.2f}"
            )

        except Exception as e:
            self.get_logger().error(f"m1_actuator: error at : {e}")
            self.coast()


    def on_cmd(self, msg: Float32):
        cmd = max(-1.0, min(1.0, msg.data))
        self._apply_raw(cmd)
        self.get_logger().debug(f"New command received: {cmd:.2f}")

    def destroy_node(self):
        try:
            self.coast()
        finally:
            self.get_logger().info("m1_actuator: shutdown.")
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = m1_actuator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("m1_actuator: keyboard interrupt.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
