#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from std_msgs.msg import String
 

class Talker(Node):

    def __init__(self):

        super().__init__('talker')

        # Создаем паблишера(publisher) на топик 'chatter' с типом сообщения String

        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Таймер срабатывает каждую секунду

        timer_period = 1.0

        self.timer = self.create_timer(timer_period, self.publish_message)

        # Создаем сообщение типа String

        self.msg = String()

        self.msg.data = "Hello, ROS2!"
 

    def publish_message(self):

        # Публикуем сообщение

        self.publisher_.publish(self.msg)

        self.get_logger().info('Publishing: "%s"' % self.msg.data)
 

def main(args=None):

    rclpy.init(args=args)

    node = Talker()

    rclpy.spin(node)

   

    # По завершении
    node.destroy_node()

    rclpy.shutdown()

 

if __name__ == '__main__':
    main()

