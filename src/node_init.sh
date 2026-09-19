# Initialise project nodes across the project-search directory
echo "Initialising project_search nodes..."

source install/setup.bash

ros2 run project_search util_weights_srv
ros2 run project_search frontier_search
ros2 run project_explore mst_planner
ros2 run project_explore nav2_handler
ros2 run project_explore exploration_manager