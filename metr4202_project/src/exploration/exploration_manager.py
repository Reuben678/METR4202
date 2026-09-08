import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class ExplorationManager(Node):

    def __init__(self):
        super().__init__('exploration_manager')

        # Subscribe to frontier goal
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )

        # Create an action client for Nav2
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.navigation_active = False

        self.get_logger().info('Exploration Manager initialised')


    def goal_callback(self, msg):
        """
        Called whenever new frontier is published
        """

        x = msg.pose.position.x
        y = msg.pose.position.y

        self.get_logger().info(
            f'Received frontier: x={x:.2f}, y={y:.2f}'
        )

        # Don't send another goal while already navigating
        if self.navigation_active:
            self.get_logger().warn(
                'Robot is already navigating. Ignoring frontier.'
            )
            return

        self.send_nav_goal(msg)


    def send_nav_goal(self, frontier):
        """
        Send the received PoseStamped frontier to Nav2.
        """

        # Wait until Nav2's NavigateToPose action server exists
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                'NavigateToPose action server not available'
            )
            return

        # Construct a Nav2 NavigateToPose goal
        nav_goal = NavigateToPose.Goal()

        nav_goal.pose = frontier

        self.get_logger().info(
            f'Sending goal to Nav2: '
            f'x={frontier.pose.position.x:.2f}, '
            f'y={frontier.pose.position.y:.2f}'
        )

        self.navigation_active = True

        # Send goal asynchronously (stops program from freezing)
        future = self.nav_client.send_goal_async(
            nav_goal,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        """
        Called when Nav2 accepts or rejects our goal.
        """

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal')
            self.navigation_active = False
            return

        self.get_logger().info('Nav2 accepted the goal')

        # Ask Nav2 to tell us when navigation finishes
        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.navigation_result_callback
        )


    def feedback_callback(self, feedback_msg):
        """
        Called while Nav2 is navigating.
        """

        feedback = feedback_msg.feedback

        self.get_logger().debug(
            f'Distance remaining: '
            f'{feedback.distance_remaining:.2f} m'
        )


    def navigation_result_callback(self, future):
        """
        Called when navigation finishes.
        """

        result = future.result()

        self.navigation_active = False

        # Need to add failure handling below here***
        self.get_logger().info(
            f'Navigation finished with status: {result.status}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = ExplorationManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()