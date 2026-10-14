#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from std_msgs.msg import String

class Listener(Node):

    def __init__(self):

        super().__init__('listener')

        # Создаем подписчика (subscriber) на топик 'chatter' с типом сообщения String

        self.subscription = self.create_subscription(

            String,

            'chatter',

            self.listener_callback,

            10)

        self.subscription  # предотвратить предупреждение о неиспользуемой переменной

 

    def listener_callback(self, msg):

        self.get_logger().info('Received: "%s"' % msg.data)

 

def main(args=None):

    rclpy.init(args=args)

    node = Listener()

    rclpy.spin(node)

   

    # По завершении
    node.destroy_node()

    rclpy.shutdown()

 

if __name__ == '__main__':

    main()
