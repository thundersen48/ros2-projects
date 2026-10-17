#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
# Импортируем сгенерированный модуль из пакета интерфейсов
from my_service_interfaces.srv import SetVelocity

class VelocityServiceClient(Node):

    def __init__(self):
        super().__init__('velocity_service_client')
        # Создаём клиент
        self.client = self.create_client(SetVelocity, 'set_velocity')

        # Ждём, пока сервер не будет доступен
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /set_velocity not available, waiting...')

    def send_request(self, linear, angular):
        req = SetVelocity.Request()
        req.linear = linear
        req.angular = angular

        self.get_logger().info(f'Sending request: linear={linear}, angular={angular}')
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error('Exception while calling service')
            return None

def main(args=None):
    rclpy.init(args=args)

    # Проверяем аргументы командной строки
    if len(sys.argv) < 3:
        print("Usage: ros2 run my_service_client client_node <linear> <angular>")
        sys.exit(1)

    try:
        linear = float(sys.argv[1])
        angular = float(sys.argv[2])
    except ValueError:
        print("Error: linear and angular velocities must be numbers.")
        sys.exit(1)

    node = VelocityServiceClient()
    response = node.send_request(linear, angular)

    if response:
        print(f"Success: {response.success}, message: '{response.message}'")
    else:
        print("No response received.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
