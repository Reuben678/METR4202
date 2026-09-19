from dataclasses import dataclass
from math import hypot, isfinite
from typing import Callable, List, Optional, Tuple
from metr4202_interfaces.msg import FrontierArray
from metr4202_interfaces.srv import GetFrontiers

import networkx as nx


@dataclass(frozen=True)
class Frontier:
    """Candidate frontier stored as an object."""
    frontier_id: int
    x: float
    y: float


@dataclass(frozen=True)
class Edge:
    """Weighted connection between two graph nodes."""
    node_a: int
    node_b: int
    cost: float


class MSTPlanner:
    """Generates an ordered frontier plan using a minimum spanning tree."""

    ROBOT_NODE_ID = -1

    def __init__(self, travel_cost_function: Optional[Callable[[Tuple[float, float], Tuple[float, float]], float]] = None) -> None:
        # Nav2 path distance can replace Euclidean distance later.
        self.travel_cost_function = (
            travel_cost_function or self.euclidean_travel_cost
        )

        self.frontier_client = self.create_client(
            GetFrontiers,
            'get_frontiers'
        )
        
    @staticmethod
    def euclidean_travel_cost(start: Tuple[float, float], goal: Tuple[float, float]) -> float:
        """Calculate straight-line travel cost."""

        return hypot(
            goal[0] - start[0],
            goal[1] - start[1],
        )

    @staticmethod
    def parse_frontiers(data: List[float], rows: int, columns: int) -> List[Frontier]:
        """Convert flattened frontier data into Frontier objects."""

        if rows < 0:
            raise ValueError(
                "The number of rows cannot be negative."
            )

        if columns < 3:
            raise ValueError(
                "Each frontier requires an ID, x position and y position."
            )

        if len(data) != rows * columns:
            raise ValueError(
                "Frontier data does not match the supplied dimensions."
            )

        frontiers: List[Frontier] = []

        for row_index in range(rows):
            start = row_index * columns
            row = data[start:start + columns]

            frontiers.append(
                Frontier(
                    frontier_id=int(row[0]),
                    x=float(row[1]),
                    y=float(row[2]),
                )
            )

        return frontiers

    def receive_frontiers(self, timout=5.0) -> List[Frontier]:
        """Request the latest frontiers from the frontier service."""
        if not self.frontier_client.service_is_ready():
            self.get_logger().warn("GetFrontiers service not available")
            return
        if self.frontier_future is not None and not self.frontier_future.done():
            return  # Request already made
        
        future = self.frontier_client.call_async(GetFrontiers.Request())
        rclpy.spin_util_future_complete(self, future, timeout)
        # Get the result of our request
        resp = future.result()
        if resp is None or not resp.success:
            self.get_logger().warn("GetFrontier returned no data")
            return None
        info = resp.frontiers
        frontiers = np.asarray(info.data, dtype=np.float32)
        frontiers.reshape(info.rows, info.cols)
        return frontiers

    def get_robot_position(self) -> Tuple[float, float]:
        """Get the current robot position from TF."""

        # Replace with the map to base_link TF lookup.
        raise NotImplementedError(
            "Robot position has not been connected to TF yet."
        )

    def construct_graph(self, frontiers: List[Frontier], robot_position: Tuple[float, float]) -> nx.Graph:
        """Construct a complete weighted graph."""

        ids = [
            frontier.frontier_id
            for frontier in frontiers
        ]

        if len(ids) != len(set(ids)):
            raise ValueError(
                "Every frontier must have a unique frontier_id."
            )

        if self.ROBOT_NODE_ID in ids:
            raise ValueError(
                f"Frontier ID {self.ROBOT_NODE_ID} is reserved for the robot."
            )

        positions = {
            self.ROBOT_NODE_ID: robot_position,
            **{
                frontier.frontier_id: (frontier.x, frontier.y)
                for frontier in frontiers
            },
        }

        graph = nx.Graph()

        for node_id, position in positions.items():
            graph.add_node(
                node_id,
                position=position,
            )

        nodes = list(positions.keys())

        for index, node_a in enumerate(nodes):
            for node_b in nodes[index + 1:]:
                cost = self.travel_cost_function(
                    positions[node_a],
                    positions[node_b],
                )

                if isfinite(cost) and cost >= 0.0:
                    graph.add_edge(
                        node_a,
                        node_b,
                        weight=cost,
                    )

        return graph

    @staticmethod
    def generate_mst(graph: nx.Graph) -> nx.Graph:
        """Generate an MST using NetworkX Kruskal's algorithm."""

        if graph.number_of_nodes() <= 1:
            return graph.copy()

        if not nx.is_connected(graph):
            raise ValueError(
                "The graph is disconnected; not every frontier is reachable."
            )

        return nx.minimum_spanning_tree(
            graph,
            weight="weight",
            algorithm="kruskal",
        )

    def recommended_traversal(self, mst: nx.Graph) -> List[int]:
        """Generate a depth-first frontier visitation order."""

        if self.ROBOT_NODE_ID not in mst:
            return []

        ordered_mst = nx.Graph()
        ordered_mst.add_nodes_from(mst.nodes(data=True))

        sorted_edges = sorted(
            mst.edges(data=True),
            key=lambda edge: edge[2]["weight"],
        )

        ordered_mst.add_edges_from(sorted_edges)

        traversal_order = list(
            nx.dfs_preorder_nodes(
                ordered_mst,
                source=self.ROBOT_NODE_ID,
            )
        )

        return [
            node_id
            for node_id in traversal_order
            if node_id != self.ROBOT_NODE_ID
        ]

    @staticmethod
    def order_frontiers(frontiers: List[Frontier], traversal_order: List[int]) -> List[Frontier]:
        """Convert ordered frontier IDs into Frontier objects."""

        frontier_by_id = {
            frontier.frontier_id: frontier
            for frontier in frontiers
        }

        return [
            frontier_by_id[frontier_id]
            for frontier_id in traversal_order
        ]

    @staticmethod
    def convert_mst_edges(mst: nx.Graph) -> List[Edge]:
        """Convert NetworkX edges into Edge objects."""

        return [
            Edge(
                node_a=node_a,
                node_b=node_b,
                cost=edge_data["weight"],
            )
            for node_a, node_b, edge_data in mst.edges(data=True)
        ]

    def plan(self, frontiers: List[Frontier], robot_position: Tuple[float, float]) -> Tuple[List[Edge], List[Frontier]]:
        """Generate an MST and ordered frontier list."""

        graph = self.construct_graph(
            frontiers,
            robot_position,
        )

        mst = self.generate_mst(graph)
        traversal_order = self.recommended_traversal(mst)

        mst_edges = self.convert_mst_edges(mst)

        ordered_frontiers = self.order_frontiers(
            frontiers,
            traversal_order,
        )

        return mst_edges, ordered_frontiers

    def generate_latest_plan(self) -> Tuple[List[Edge], List[Frontier]]:
        """Request current data and generate a new MST plan."""

        frontiers = self.receive_frontiers()
        if frontiers is None:
            # Something went wrong, so raise exception
            raise ValueError("MST could not gather frontiers")
        robot_position = self.get_robot_position()

        return self.plan(
            frontiers,
            robot_position,
        )


def main() -> None:
    """Test the MST Planner."""

    robot_position = (0.0, 0.0)

    test_frontiers = [
        Frontier(frontier_id=0, x=2.0, y=1.0),
        Frontier(frontier_id=1, x=5.0, y=1.5),
        Frontier(frontier_id=2, x=4.0, y=5.0),
        Frontier(frontier_id=3, x=1.0, y=4.0),
    ]

    planner = MSTPlanner()

    mst_edges, ordered_frontiers = planner.plan(
        test_frontiers,
        robot_position,
    )

    print("\nMinimum spanning tree:")

    for edge in mst_edges:
        print(
            f"  {edge.node_a} -> {edge.node_b}, "
            f"travel cost = {edge.cost:.2f}"
        )

    print("\nRecommended frontier order:")

    for frontier in ordered_frontiers:
        print(
            f"  Frontier {frontier.frontier_id}: "
            f"({frontier.x:.2f}, {frontier.y:.2f})"
        )


if __name__ == "__main__":
    main()