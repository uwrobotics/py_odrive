import rclpy
from ros2_lifecycle_py.lifecycle import LifecycleNode
from ros2_lifecycle_py.lifecycle import TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor


class MyLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('my_lifecycle_node')
        self.get_logger().info('Lifecycle Node Created')

    def on_configure(self, state):
        ''' can instance activation '''
        ''' Calibration and direction testing '''
        self.get_logger().info('Configuring...')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        ''' start the listener -> send '''
        ''' Need state machine for managing control loop state '''
        self.get_logger().info('Activating...')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        ''' dump log '''
        self.get_logger().info('Deactivating...')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        ''' close can instance '''
        self.get_logger().info('Cleaning up...')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS


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