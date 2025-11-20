#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from service_package.srv import SetSpeed
from std_msgs.msg import Float32, String


class SpeedService(Node):
    def __init__(self):
        super().__init__('pct_speed_service')

        # Crea el servicio SetSpeed
        self.srv = self.create_service(SetSpeed, 'set_speed', self.handle_set_speed)

        # Publishers: uno para la velocidad y otro para mensajes de estado
        self.pub_speed = self.create_publisher(Float32, 'max_speed_pct', 10)
        self.pub_status = self.create_publisher(String, 'status_text', 10)

        self.current_pct_speed = 0.0
        self.get_logger().info('Servicio "set_speed" inicializado.')

    def handle_set_speed(self, request, response):
        self.current_pct_speed = request.pct_speed

        # Publicar nueva velocidad
        msg_speed = Float32()
        msg_speed.data = self.current_pct_speed
        self.pub_speed.publish(msg_speed)

        # Publicar texto de estado
        msg_status = String()
        if self.current_pct_speed == 0.0:
            msg_status.data = 'motor stopped.'
        else:
            msg_status.data = f'motor at {self.current_pct_speed:.1f}% speed.'

        self.pub_status.publish(msg_status)

        # Responder al cliente
        response.success = True
        response.message = msg_status.data
        self.get_logger().info(f"Msg: {msg_status.data}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = SpeedService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
