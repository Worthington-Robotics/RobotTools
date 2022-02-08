import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from autobt_msgs.srv import StringService


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv1 = self.create_service(Trigger, 'dummy_srv_trig', self.add_two_ints_callback)
        self.srv2 = self.create_service(StringService, 'dummy_srv_str', self.string_cb)

    def string_cb(self, request, response):

        self.get_logger().info('Got a request! {}'.format(request.request_string))

        return response

    def add_two_ints_callback(self, request, response):

        self.get_logger().info('Got a request!')

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()