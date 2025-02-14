from uwrt_ros_msg.srv import OdriveCmd

import rclpy
from rclpy.node import Node


class OdriveMsgService(Node):

    def __init__(self):
        super().__init__('odrive_msg_service')
        self.srv = self.create_service(OdriveCmd, 'OdriveCmd', self.odrive_cmd_callback)

    def odrive_cmd_callback(self, request, response):
        print(request.axis_id, request.cmd, request.payload)
        response.status = True
        return response


def main():
    rclpy.init()

    msg_service = OdriveMsgService()

    rclpy.spin(msg_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()