import rclpy

'''
lifecycle will be implemented externally application is just listener
'''


def main():
    rclpy.init()
    node = MyLifecycleNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()