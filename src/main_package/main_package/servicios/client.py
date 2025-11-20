#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from service_package.srv import SetSpeed

class SpeedClient(Node):
    def __init__(self):
        super().__init__('pct_speed_client')
        self.cli = self.create_client(SetSpeed, 'set_speed')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio "set_speed"...')

        self.req = SetSpeed.Request()

    def send_request(self, pct_speed):
        self.req.pct_speed = pct_speed
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = SpeedClient()

    try:
        pct = float(input('Ingrese porcentaje de velocidad (0 a 100): '))
        if 0 <= pct <= 100:
            response = node.send_request(pct)
            print('\nRespuesta del servidor:')
            print(f' success = {response.success}')
            print(f' message = "{response.message}"')
        else:
            print('Porcentaje fuera de rango (0–100)')
    except Exception as e:
        print(f'Error: {e}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
