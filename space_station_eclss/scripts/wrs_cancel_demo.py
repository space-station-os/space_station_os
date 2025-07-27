import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from space_station_eclss.action import WaterRecovery

class WRSClient(Node):
    def __init__(self):
        super().__init__('wrs_cancel_demo')
        self.client = ActionClient(self, WaterRecovery, '/water_recovery_systems')

    def send_goal(self):
        self.client.wait_for_server()
        goal_msg = WaterRecovery.Goal()
        goal_msg.urine_volume = 50.0
        self._goal_future = self.client.send_goal_async(goal_msg)
        self._goal_future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted. Waiting 2 seconds before cancel...')
        self.create_timer(2.0, self.cancel_goal)

    def cancel_goal(self):
        self.get_logger().info('Sending cancel request...')
        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_response)

    def cancel_response(self, future):
        cancel_result = future.result()
        self.get_logger().info(f'Cancel response: {cancel_result}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = WRSClient()
    node.send_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
