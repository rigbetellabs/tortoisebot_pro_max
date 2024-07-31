#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from rclpy.logging import LoggingSeverity

class LogSubscriber(Node):
    def __init__(self):
        super().__init__('rbl_logger')
        self.subscription = self.create_subscription(
            Log,
            'rbl_logger',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("RBL Logger Initialized")

    def listener_callback(self, msg):
        #print("Received log message:", msg)  # Debug print

        # Directly use msg.level as the severity level
        severity = LoggingSeverity(msg.level)
        #print("Mapped severity level:", severity)  # Debug print

        # Log the message with the determined severity level using dedicated functions
        if severity == 10:
            self.log_debug(msg.msg)
        elif severity == 20:
            self.log_info(msg.msg)
        elif severity == 30:
            self.log_warn(msg.msg)
        elif severity == 40:
            self.log_error(msg.msg)
        elif severity == 50:
            self.log_fatal(msg.msg)

    def log_debug(self, message):
        self.get_logger().debug(message)

    def log_info(self, message):
        self.get_logger().info(message)

    def log_warn(self, message):
        self.get_logger().warning(message)

    def log_error(self, message):
        self.get_logger().error(message)

    def log_fatal(self, message):
        self.get_logger().fatal(message)

def main(args=None):
    rclpy.init(args=args)
    log_subscriber = LogSubscriber()
    rclpy.spin(log_subscriber)
    log_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()











