from dataclasses import dataclass
from math import hypot, isfinite
from typing import Callable, List, Optional, Tuple


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


class UnionFind:
    """Tracks connected components for Kruskal's algorithm."""

    def __init__(self, nodes: List[int]) -> None:
        self.parent = {node: node for node in nodes}
        self.rank = {node: 0 for node in nodes}

    def find(self, node: int) -> int:
        if self.parent[node] != node:
            self.parent[node] = self.find(self.parent[node])
        return self.parent[node]

    def union(self, node_a: int, node_b: int) -> bool:
        root_a = self.find(node_a)
        root_b = self.find(node_b)

        if root_a == root_b:
            return False

        if self.rank[root_a] < self.rank[root_b]:
            root_a, root_b = root_b, root_a

        self.parent[root_b] = root_a

        if self.rank[root_a] == self.rank[root_b]:
            self.rank[root_a] += 1

        return True


class MSTPlanner:
    """
    Constructs a frontier graph, calculates an MST and recommends
    a traversal order rooted at the robot's current position.
    """

    ROBOT_NODE_ID = -1

    def __init__(self, travel_cost_function: Optional[Callable[[Tuple[float, float], Tuple[float, float]], float]] = None) -> None:
        # Possible improvement: request path from Nav2 ComputePathToPose

        self.travel_cost_function = (
            travel_cost_function or self.euclidean_travel_cost
        )

    @staticmethod
    def euclidean_travel_cost(start: Tuple[float, float], goal: Tuple[float, float]) -> float:
        """Temporary travel-cost estimate."""
        return hypot(goal[0] - start[0], goal[1] - start[1])

    @staticmethod
    def parse_frontiers(data: List[float], rows: int, columns: int) -> List[Frontier]:
        """Convert flattened frontier data into Frontier objects."""

        if rows < 0:
            raise ValueError("The number of rows cannot be negative.")

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

    def receive_frontiers(self) -> List[Frontier]:
        """Request the latest frontiers from the frontier service."""

        # Replace with the frontier-search service request.
        raise NotImplementedError(
            "Frontier input has not been connected to ROS yet."
        )

    def get_robot_position(self) -> Tuple[float, float]:
        """Get the current robot position from TF."""

        # Replace with the map to base_link TF lookup.
        raise NotImplementedError(
            "Robot position has not been connected to TF yet."
        )

    def construct_graph(self, frontiers: List[Frontier], robot_position: Tuple[float, float]) -> Tuple[List[int], List[Edge]]:
        """Construct a complete weighted graph."""

        ids = [frontier.frontier_id for frontier in frontiers]

        if len(ids) != len(set(ids)):
            raise ValueError("Every frontier must have a unique frontier_id.")

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

        nodes = list(positions.keys())
        edges: List[Edge] = []

        for index, node_a in enumerate(nodes):
            for node_b in nodes[index + 1:]:
                cost = self.travel_cost_function(
                    positions[node_a],
                    positions[node_b],
                )

                # Infinite cost can later represent an unreachable Nav2 path.
                if isfinite(cost) and cost >= 0.0:
                    edges.append(Edge(node_a, node_b, cost))

        return nodes, edges

    @staticmethod
    def generate_mst(nodes: List[int], edges: List[Edge]) -> List[Edge]:
        """Generate a minimum spanning tree using Kruskal's algorithm."""

        if len(nodes) <= 1:
            return []

        union_find = UnionFind(nodes)
        mst_edges: List[Edge] = []

        for edge in sorted(edges, key=lambda item: item.cost):
            if union_find.union(edge.node_a, edge.node_b):
                mst_edges.append(edge)

            if len(mst_edges) == len(nodes) - 1:
                break

        if len(mst_edges) != len(nodes) - 1:
            raise ValueError(
                "The graph is disconnected; not every frontier is reachable."
            )

        return mst_edges

    def recommended_traversal(self, mst_edges: List[Edge]) -> List[int]:
        """
        Produce a depth-first frontier visitation order.

        Lower-cost neighbouring branches are visited first.
        The robot node is omitted from the returned list.
        """

        adjacency = {}

        for edge in mst_edges:
            adjacency.setdefault(edge.node_a, []).append(
                (edge.node_b, edge.cost)
            )
            adjacency.setdefault(edge.node_b, []).append(
                (edge.node_a, edge.cost)
            )

        traversal_order: List[int] = []
        visited = set()

        def depth_first_search(node: int) -> None:
            visited.add(node)

            neighbours = sorted(
                adjacency.get(node, []),
                key=lambda neighbour: neighbour[1],
            )

            for neighbour_id, _ in neighbours:
                if neighbour_id not in visited:
                    if neighbour_id != self.ROBOT_NODE_ID:
                        traversal_order.append(neighbour_id)

                    depth_first_search(neighbour_id)

        depth_first_search(self.ROBOT_NODE_ID)
        return traversal_order

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

    def plan(self, frontiers: List[Frontier], robot_position: Tuple[float, float]) -> Tuple[List[Edge], List[Frontier]]:
        """Run the MST planning process."""

        nodes, graph_edges = self.construct_graph(
            frontiers,
            robot_position,
        )

        mst_edges = self.generate_mst(nodes, graph_edges)
        traversal_order = self.recommended_traversal(mst_edges)

        ordered_frontiers = self.order_frontiers(
            frontiers,
            traversal_order,
        )

        return mst_edges, ordered_frontiers

    def generate_latest_plan(self) -> Tuple[List[Edge], List[Frontier]]:
        """Request current data and generate a new MST plan."""

        frontiers = self.receive_frontiers()
        robot_position = self.get_robot_position()

        return self.plan(
            frontiers,
            robot_position,
        )


def main() -> None:
    """Temporary test data until the ROS interfaces are connected."""

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