# Initialise project nodes across the project-search directory
echo "Initialising project-search nodes..."

ros2 run project-search util_weights_srv
ros2 run project-search frontier_search
