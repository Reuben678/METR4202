from enum import Enum, auto
from typing import Callable, Optional

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class NavigationResult(Enum):
    """Possible navigation results."""
    SUCCEEDED = auto()
    FAILED = auto()
    CANCELLED = auto()


class Nav2Handler:
    """Sends navigation goals using Nav2 Simple Commander."""

    def __init__(self) -> None:
        self.navigator = BasicNavigator()

        self.navigation_active = False
        self.result_callback: Optional[
            Callable[[NavigationResult], None]
        ] = None

    def wait_until_active(self) -> None:
        """Wait until Nav2 is active."""

        self.navigator.waitUntilNav2Active()

    def navigate_to(self, x: float, y: float, result_callback: Callable[[NavigationResult], None]) -> bool:
        """Send a navigation goal to Nav2."""

        if self.navigation_active:
            self.navigator.get_logger().warning(
                "A navigation goal is already active."
            )
            return False

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = (
            self.navigator.get_clock().now().to_msg()
        )

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        self.result_callback = result_callback
        self.navigation_active = True

        self.navigator.goToPose(goal)

        self.navigator.get_logger().info(
            f"Navigation goal sent: ({x:.2f}, {y:.2f})"
        )

        return True

    def update(self) -> None:
        """Check navigation feedback and completion."""

        if not self.navigation_active:
            return

        if not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()

            if feedback is not None:
                self.navigator.get_logger().debug(
                    f"Distance remaining: "
                    f"{feedback.distance_remaining:.2f} m"
                )

            return

        task_result = self.navigator.getResult()

        if task_result == TaskResult.SUCCEEDED:
            result = NavigationResult.SUCCEEDED

            self.navigator.get_logger().info(
                "Destination reached."
            )

        elif task_result == TaskResult.CANCELED:
            result = NavigationResult.CANCELLED

            self.navigator.get_logger().warning(
                "Navigation was cancelled."
            )

        else:
            result = NavigationResult.FAILED

            self.navigator.get_logger().warning(
                "Navigation failed."
            )

        self.finish_navigation(result)

    def finish_navigation(self, result: NavigationResult) -> None:
        """Report the result to the Exploration Manager."""

        self.navigation_active = False

        callback = self.result_callback
        self.result_callback = None

        if callback is not None:
            callback(result)

    def cancel_goal(self) -> None:
        """Cancel the current navigation goal."""

        if not self.navigation_active:
            return

        self.navigator.get_logger().warning(
            "Cancelling current navigation goal."
        )

        self.navigator.cancelTask()