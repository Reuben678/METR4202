from enum import Enum, auto
from math import hypot
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from mst_planner import Frontier, MSTPlanner
from nav2_handler import NavigationResult, Nav2Handler
from frontier_msgs.msg import FrontierArray


class ExplorationState(Enum):
    """Possible exploration manager states."""
    INITIALISING = auto()
    REQUESTING_PLAN = auto()
    SELECTING_GOAL = auto()
    NAVIGATING = auto()
    UPDATING_RECORDS = auto()
    COMPLETE = auto()


class ExplorationManager(Node):
    """Requests MST plans and manages frontier navigation."""

    def __init__(self) -> None:
        super().__init__("exploration_manager")

        self.state = ExplorationState.INITIALISING

        self.mst_planner = MSTPlanner()
        self.nav2_handler = Nav2Handler()

        self.ordered_frontiers: List[Frontier] = []
        self.visited_positions: List[Tuple[float, float]] = []
        self.failed_positions: List[Tuple[float, float]] = []

        self.current_goal: Optional[Frontier] = None
        self.last_navigation_result: Optional[NavigationResult] = None

        self.position_tolerance = 0.30
        self.navigation_started = False
        self.mission_complete = False

        self.timer = self.create_timer(
            0.10,
            self.step,
        )        

    def dependencies_available(self) -> bool:
        """Check whether Nav2 is ready."""

        self.nav2_handler.wait_until_active()
        return True

    def request_plan(self) -> List[Frontier]:
        """Request a new ordered plan from the MST Planner."""

        _, ordered_frontiers = (
            self.mst_planner.generate_latest_plan()
        )

        return ordered_frontiers

    @staticmethod
    def distance(position_a: Tuple[float, float], position_b: Tuple[float, float]) -> float:
        """Calculate the distance between two positions."""

        return hypot(
            position_b[0] - position_a[0],
            position_b[1] - position_a[1],
        )

    def position_recorded(self, frontier: Frontier, recorded_positions: List[Tuple[float, float]]) -> bool:
        """Check whether a frontier position is recorded."""

        frontier_position = (
            frontier.x,
            frontier.y,
        )

        return any(
            self.distance(frontier_position, position)
            <= self.position_tolerance
            for position in recorded_positions
        )

    def select_goal(self, ordered_frontiers: List[Frontier]) -> Optional[Frontier]:
        """Select the first unvisited and unfailed frontier."""

        for frontier in ordered_frontiers:
            if self.position_recorded(
                frontier,
                self.visited_positions,
            ):
                continue

            if self.position_recorded(
                frontier,
                self.failed_positions,
            ):
                continue

            return frontier

        return None

    def start_navigation(self) -> None:
        """Send the selected goal to the Nav2 Handler."""

        if self.current_goal is None:
            self.state = ExplorationState.REQUESTING_PLAN
            return

        self.get_logger().info(
            f"Sending frontier {self.current_goal.frontier_id}: "
            f"({self.current_goal.x:.2f}, "
            f"{self.current_goal.y:.2f})"
        )

        goal_sent = self.nav2_handler.navigate_to(
            self.current_goal.x,
            self.current_goal.y,
            self.navigation_finished,
        )

        if goal_sent:
            self.navigation_started = True
        else:
            self.last_navigation_result = NavigationResult.FAILED
            self.state = ExplorationState.UPDATING_RECORDS

    def navigation_finished(self, result: NavigationResult) -> None:
        """Receive the completed navigation result."""

        self.last_navigation_result = result
        self.navigation_started = False
        self.state = ExplorationState.UPDATING_RECORDS

    def update_records(self) -> None:
        """Record the result of the navigation attempt."""

        if self.current_goal is None:
            return

        goal_position = (
            self.current_goal.x,
            self.current_goal.y,
        )

        if self.last_navigation_result == NavigationResult.SUCCEEDED:
            self.visited_positions.append(goal_position)

            self.get_logger().info(
                f"Reached frontier "
                f"{self.current_goal.frontier_id}."
            )

        else:
            self.failed_positions.append(goal_position)

            self.get_logger().warning(
                f"Failed to reach frontier "
                f"{self.current_goal.frontier_id}."
            )

        self.current_goal = None
        self.last_navigation_result = None

    def publish_mission_status(self) -> None:
        """Publish the final mission status."""

        self.get_logger().info(
            "Exploration complete."
        )

    def step(self) -> None:
        """Execute one exploration state."""

        if self.state == ExplorationState.INITIALISING:
            if self.dependencies_available():
                self.get_logger().info(
                    "Required dependencies are available."
                )
                self.state = ExplorationState.REQUESTING_PLAN

        elif self.state == ExplorationState.REQUESTING_PLAN:
            try:
                self.ordered_frontiers = self.request_plan()
                self.state = ExplorationState.SELECTING_GOAL

            except (ValueError, RuntimeError) as error:
                self.get_logger().error(
                    f"MST planning failed: {error}"
                )
                self.state = ExplorationState.COMPLETE

        elif self.state == ExplorationState.SELECTING_GOAL:
            self.current_goal = self.select_goal(
                self.ordered_frontiers
            )

            if self.current_goal is None:
                self.state = ExplorationState.COMPLETE
            else:
                self.navigation_started = False
                self.state = ExplorationState.NAVIGATING

        elif self.state == ExplorationState.NAVIGATING:
            if not self.navigation_started:
                self.start_navigation()
            else:
                self.nav2_handler.update()

        elif self.state == ExplorationState.UPDATING_RECORDS:
            self.update_records()

            # Request a new MST after every navigation result.
            self.state = ExplorationState.REQUESTING_PLAN

        elif self.state == ExplorationState.COMPLETE:
            if not self.mission_complete:
                self.publish_mission_status()
                self.mission_complete = True
                self.timer.cancel()


def main(args=None) -> None:
    """Run the Exploration Manager."""

    rclpy.init(args=args)

    manager = ExplorationManager()

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()