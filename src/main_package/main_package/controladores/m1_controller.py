#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8MultiArray


class m1_controller(Node):
    def __init__(self):
        super().__init__('m1_controller_node')

        # declare param
        self.frequency_hz = 20.0 
        self.ramp_step = 5      
        self.pwm_max = 255

        # aditional config
        self.max_pct = 0.0           # % de velocidad deseado [-100, 100]
        self.current_pwm = 0            # Valor actual de PWM [0–255]
        self.max_pwm = 0             # Valor objetivo de PWM [0–255]
        self.direction = 1              # 1 = adelante, -1 = reversa
        
        # Create publisher
        self.pub_motor_cmd = self.create_publisher(Float32, 'motor_cmd', 10)
        # Create subscriptions  
        self.sub = self.create_subscription(Float32,'max_speed_pct',self.callback_max_speed,10) #pct from server
        self.sub1 = self.create_subscription(Int8MultiArray, "tcrt5000_topic", self.callback_tcrt5000, 10) # ir sensors

        # Control loop clock
        self.timer = self.create_timer(1.0 / self.frequency_hz, self.control_loop)

        self.get_logger().info(
            f"m1_controller: ready | freq={self.frequency_hz:.1f} Hz | ramp_step={self.ramp_step}."
        )
    ### Methods
    # CALLBACKS
    def callback_max_speed(self, msg: Float32):
        self.max_pct = max(-100.0, min(msg.data, 100.0))
        self.direction = 1 if self.target_pct >= 0 else -1
        self.max_pwm = round((abs(self.target_pct) / 100.0) * self.pwm_max)
        self.get_logger().info(f"m1_controller: Max_speed received: {self.target_pct:.1f}%")
    
    def callback_tcrt5000(self, msg: Int8MultiArray):
        self.sensor_values = msg.data
        self.get_logger().debug(f"m1_controller: TCRT5000 readings: {self.sensor_values}")

    def control_loop(self):
        # TODO define control loop
        if self.current_pwm < self.target_pwm:
            self.current_pwm = min(self.current_pwm + self.ramp_step, self.target_pwm)
        elif self.current_pwm > self.target_pwm:
            self.current_pwm = max(self.current_pwm - self.ramp_step, self.target_pwm)

        
        # Normalización del comando [-1.0, 1.0]
        normalized = (self.current_pwm / self.pwm_max) * self.direction
        # Publicar comando al actuador
        msg = Float32()
        msg.data = normalized
        self.pub_motor_cmd.publish(msg)
        self.get_logger().debug(
            f"PWM={self.current_pwm:3d}/{self.pwm_max} Dir={self.direction:+d} Cmd={normalized:+.2f}"
        )

    def destroy_node(self):
        try:
            # Publica 0 al detenerse
            msg = Float32()
            msg.data = 0.0
            self.pub_motor_cmd.publish(msg)
            self.get_logger().info("Motor stopped (cmd=0.0).")
        finally:
            self.get_logger().info("m1_controller: shutdown.")
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = m1_controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("m1_controller: keyboard interrupt.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
