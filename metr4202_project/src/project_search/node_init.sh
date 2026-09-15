# Initialise project nodes across the project-search directory
echo "Initialising project_search nodes..."

source ~/.bashrc

ros2 run project_search util_weights_srv
ros2 run project_search frontier_search
