import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
from RPi import GPIO
import time


class tcrt5000(Node):
    def __init__(self):
        super().__init__("tcrt5000_node")

        # declare param
        self.declare_parameter("pinFarLeft")
        self.declare_parameter("pinLeft")
        self.declare_parameter("pinCenter")
        self.declare_parameter("pinRight")
        self.declare_parameter("pinFarRight")

        # Verify all param arrived
        self._pins = [
                int(self.get_parameter("pinFarLeft").value),
                int(self.get_parameter("pinLeft").value),
                int(self.get_parameter("pinCenter").value),
                int(self.get_parameter("pinRight").value),
                int(self.get_parameter("pinFarRight").value)
            ]
        # If any param missing, raise error
        if not all(isinstance(p, int) for p in self._pins):
            raise ValueError("param missing in launch file: pinFarLeft, pinLeft, pinCenter, pinRight, pinFarRight.")

        # init hardware
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        for pin in self._pins:
            GPIO.setup(pin, GPIO.IN)
        time.sleep(0.5)

        # create publisher
        self.publisher_ = self.create_publisher(Int8MultiArray, "tcrt5000_topic", 10)

        # sensing clock
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("TCRT5000: Ready.")

    # Methodos
    def _read_sensors(self):
        return [GPIO.input(pin) for pin in self._pins]

    def timer_callback(self):
        readings = self._read_sensors()
        msg = Int8MultiArray()
        msg.data = readings
        self.publisher_.publish(msg)
        self.get_logger().debug(f"TCRT5000: readings: {readings}")

    def destroy_node(self):
        GPIO.cleanup()
        self.get_logger().info("TCRT5000: shutdown")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = tcrt5000()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("TCRT5000: keyboard interrupt.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
