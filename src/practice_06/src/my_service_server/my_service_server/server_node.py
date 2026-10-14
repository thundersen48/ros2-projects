#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_service_interfaces.srv import SetVelocity
from geometry_msgs.msg import Twist          # новое: для управления скоростью

class VelocityServiceServer(Node):
    def __init__(self):
        super().__init__('velocity_service_server')
        
        # Создаём издателя в топик управления черепашкой
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )
        
        # Создаём службу, как и раньше
        self.srv = self.create_service(
            SetVelocity,
            'set_velocity',
            self.handle_set_velocity
        )
        self.get_logger().info('Service /set_velocity is up and running!')

    def handle_set_velocity(self, request, response):
        linear = request.linear
        angular = request.angular
        self.get_logger().info(
            f'Received velocity request: linear={linear}, angular={angular}'
        )
        
        # Формируем сообщение Twist и публикуем его
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        self.cmd_publisher.publish(twist_msg)
        
        # Заполняем ответ (заглушка теперь не нужна, но оставим для обратной связи)
        response.success = True
        response.message = f'Velocity set: linear={linear:.2f}, angular={angular:.2f}'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = VelocityServiceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()