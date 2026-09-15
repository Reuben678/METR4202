from util_weights.srv import UtilWeights

import rclpy
from rclpy import Node

from nav_msgs.msg import OccupancyGrid

import numpy as np

# Trinary occupancy grid values
FREE, UNKNOWN, OCCUPIED = 0, -1, 100

class WeightCalc (Node):
    def __init__(self):
        super().__init__('WeightCalculation')

        self.srv = self.create_service(
            UtilWeights,
            'util_weights_service',
            self.calc_weights_callback
            )

        self.latest_map = None
        self.w_info = 1.0
        self.w_cost = 1.0

        self.grid_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.occpancy_grid_callback,
            1
        )

        # Callback functions
        #____________________________________________________________
        def occupancy_grid_callback(self, msg:OccupancyGrid):
            self.latest_map = msg
            self.process_grid()
            return

        def calc_weights_callback(self, response):
            
            response.w_info = self.w_info
            response.w_cost = self.w_cost

            self.get_logger().info(f"Weights provided were {w_info} & {w_cost}")

            return response
        
        # Process occupancy grid data
        def process_grid(self):
            if self.latest_map is None:
                self.get_logger().debug("No new map data received")
                return

            # Process latest msg
            msg = self.latest
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution

            # Occupancy grid array in 2D
            grid = np.array(msg.data, dtype=np.int8).reshape((height, width))
            free, unknown, occupied = count_cells(grid)
            calc_util_weights(free, unknown, occupied)

        def count_cells(self, grid):
            free_mask = grid == FREE
            unknown_mask = grid == UNKNOWN
            occupied_mask = grid == OCCUPIED

            free = np.count_nonzero(free_mask)
            occupied = np.count_nonzero(occupied_mask)
            unknown = np.count_nonzero(unknown_mask)

            return free, unknown, occupied

        # Calculate the weights of information gain and cost
        def calc_util_weights(self, free, unknown, occupied):
            w_info = unknown / (free + occupied)
            w_cost = 1 - w_info
            self.w_info
            self.w_cost

    def main():
        rclpy.init()

        weight_calc = WeightCalc()

        rclpy.spin(weight_calc)

        rclpy.shutdown()

    if __name__ == '__main__':
        main()
