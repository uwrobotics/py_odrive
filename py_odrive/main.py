from uwrt_ros_msg.srv import OdriveCmd

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(OdriveCmd, 'OdriveCmd', self.odrive_cmd_callback)

    def odrive_cmd_callback(self, request, response):
        print(request.axis_id, request.cmd, request.payload)
        response.status = True

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()