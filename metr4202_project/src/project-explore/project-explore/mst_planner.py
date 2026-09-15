from dataclasses import dataclass
from math import hypot, isfinite
from typing import Callable, List, Optional, Tuple


@dataclass(frozen=True)
class Frontier:
    """A candidate frontier in the map frame."""
    frontier_id: int
    x: float
    y: float


@dataclass(frozen=True)
class Edge:
    """A weighted connection between two graph nodes."""
    node_a: int
    node_b: int
    cost: float


class UnionFind:
    """Tracks connected components for Kruskal's MST algorithm."""

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

    def __init__(
        self,
        travel_cost_function: Optional[
            Callable[[Tuple[float, float], Tuple[float, float]], float]
        ] = None,
    ) -> None:
        # Replace this function later with a Nav2 path-cost request.
        self.travel_cost_function = (
            travel_cost_function or self.euclidean_travel_cost
        )

    @staticmethod
    def euclidean_travel_cost(
        start: Tuple[float, float],
        goal: Tuple[float, float],
    ) -> float:
        """Temporary travel-cost estimate."""
        return hypot(goal[0] - start[0], goal[1] - start[1])

    def receive_frontiers(self) -> List[Frontier]:
        """
        Placeholder for the future ROS topic/service input.

        Later, this can convert received frontier messages into
        a list of Frontier objects.
        """
        raise NotImplementedError(
            "Frontier input has not been connected to ROS yet."
        )

    def construct_graph(
        self,
        frontiers: List[Frontier],
        robot_position: Tuple[float, float],
    ) -> Tuple[List[int], List[Edge]]:
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

    def recommended_traversal(
        self,
        mst_edges: List[Edge],
    ) -> List[int]:
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

    def plan(
        self,
        frontiers: List[Frontier],
        robot_position: Tuple[float, float],
    ) -> Tuple[List[Edge], List[int]]:
        """Run the complete MST planning process."""

        nodes, graph_edges = self.construct_graph(
            frontiers,
            robot_position,
        )

        mst_edges = self.generate_mst(nodes, graph_edges)
        traversal_order = self.recommended_traversal(mst_edges)

        return mst_edges, traversal_order


def main() -> None:
    """Temporary test data until the ROS frontier input is connected."""

    robot_position = (0.0, 0.0)

    test_frontiers = [
        Frontier(frontier_id=0, x=2.0, y=1.0),
        Frontier(frontier_id=1, x=5.0, y=1.5),
        Frontier(frontier_id=2, x=4.0, y=5.0),
        Frontier(frontier_id=3, x=1.0, y=4.0),
    ]

    planner = MSTPlanner()

    mst_edges, traversal_order = planner.plan(
        test_frontiers,
        robot_position,
    )

    print("\nMinimum spanning tree:")

    for edge in mst_edges:
        print(
            f"  {edge.node_a} -> {edge.node_b}, "
            f"travel cost = {edge.cost:.2f}"
        )

    print(f"\nRecommended frontier order: {traversal_order}")


if __name__ == "__main__":
    main()