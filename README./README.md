# METR4202 — Autonomous Exploration & Target Search

## Project Overview

Develop an autonomous TurtleBot3 Waffle Pi system that can explore an unknown environment, build a map, navigate autonomously, and locate ArUco targets.

---

# Project Roadmap

## Phase 1 — Project Setup

- [ ] Set up GitHub repository and ROS 2 workspace.
- [ ] Establish project structure and development workflow.
- [ ] Configure TurtleBot3 Waffle Pi.
- [ ] Confirm the team can build and run the project.

## Phase 2 — Simulation Setup

- [ ] Set up TurtleBot3 Waffle Pi in Gazebo.
- [ ] Create/configure the simulated environment.
- [ ] Verify robot movement.
- [ ] Verify LiDAR, camera, odometry, and TF.
- [ ] Confirm the simulation runs reliably.

## Phase 3 — Mapping

- [ ] Set up SLAM Toolbox.
- [ ] Connect LiDAR and odometry to SLAM.
- [ ] Generate an occupancy-grid map.
- [ ] Visualise and validate the map in RViz2.
- [ ] Confirm the required map resolution.

## Phase 4 — Autonomous Navigation

- [ ] Set up and configure Nav2.
- [ ] Configure planners, controllers, and costmaps.
- [ ] Test navigation between manually selected waypoints.
- [ ] Verify obstacle avoidance.
- [ ] Test navigation failures and recovery.

## Phase 5 — Autonomous Waypoints

- [ ] Create a ROS 2 node for sending navigation goals.
- [ ] Connect the node to Nav2.
- [ ] Send waypoints automatically.
- [ ] Monitor navigation results.
- [ ] Handle successful and failed navigation attempts.

## Phase 6 — Exploration Algorithm

- [ ] Process the SLAM occupancy grid.
- [ ] Identify unexplored areas/frontiers.
- [ ] Generate possible exploration locations.
- [ ] Rank/select the best exploration location.
- [ ] Convert the selected location into a safe waypoint.

## Phase 7 — Autonomous Exploration

- [ ] Connect the exploration algorithm to Nav2.
- [ ] Automatically navigate between exploration waypoints.
- [ ] Continuously update the map.
- [ ] Detect new unexplored areas.
- [ ] Avoid repeatedly visiting the same areas.
- [ ] Continue until the exploration/search condition is met.

## Phase 8 — Failure Handling

- [ ] Detect failed navigation plans.
- [ ] Detect failed navigation execution.
- [ ] Handle unreachable exploration points.
- [ ] Prevent repeated failed attempts.
- [ ] Select alternative exploration locations.
- [ ] Implement appropriate recovery behaviour.

## Phase 9 — ArUco Target Detection

- [ ] Configure the TurtleBot3 camera.
- [ ] Implement ArUco marker detection.
- [ ] Identify marker IDs.
- [ ] Estimate the target position relative to the robot.
- [ ] Transform detections into the map coordinate system.
- [ ] Store detected target locations.

## Phase 10 — Target Localisation

- [ ] Validate target position estimates.
- [ ] Test targets at different distances and angles.
- [ ] Measure localisation error.
- [ ] Achieve the required approximately 0.5 m accuracy.
- [ ] Handle repeated detections of the same target.

## Phase 11 — Full System Integration

- [ ] Combine SLAM, navigation, exploration, and target detection.
- [ ] Launch the complete system together.
- [ ] Explore while continuously searching for targets.
- [ ] Record target locations on the map.
- [ ] Continue exploring after targets are detected.
- [ ] Ensure all components communicate correctly.

## Phase 12 — Search Completion & Coverage

- [ ] Define when the search is considered complete.
- [ ] Determine whether all required targets have been found.
- [ ] Determine whether useful unexplored areas remain.
- [ ] Implement the final stopping condition.
- [ ] Evaluate how much of the environment was explored.
- [ ] Assess whether complete coverage can be guaranteed.

## Phase 13 — Testing & Evaluation

- [ ] Test different starting positions.
- [ ] Test different environments.
- [ ] Test different target locations.
- [ ] Test multiple targets.
- [ ] Test navigation failures.
- [ ] Measure exploration time and distance travelled.
- [ ] Measure area/percentage explored.
- [ ] Measure target localisation accuracy.
- [ ] Record and analyse results.

## Phase 14 — Optimisation

- [ ] Improve frontier selection.
- [ ] Improve exploration efficiency.
- [ ] Reduce unnecessary travel.
- [ ] Improve navigation reliability.
- [ ] Improve target detection.
- [ ] Improve target localisation.
- [ ] Improve overall system robustness.

## Phase 15 — Final Demonstration

- [ ] Finalise the complete ROS 2 system.
- [ ] Create a reliable launch procedure.
- [ ] Perform a complete end-to-end test.
- [ ] Demonstrate autonomous mapping.
- [ ] Demonstrate autonomous exploration.
- [ ] Demonstrate target detection and localisation.
- [ ] Demonstrate failure handling.
- [ ] Demonstrate search completion.
- [ ] Finalise documentation, report, and presentation.

---

# Overall Workflow

```text
Setup
  ↓
Gazebo + TurtleBot3
  ↓
Sensors + TF
  ↓
SLAM / Mapping
  ↓
Nav2
  ↓
Autonomous Waypoints
  ↓
Frontier Detection
  ↓
Exploration
  ↓
ArUco Detection
  ↓
Target Localisation
  ↓
Full Integration
  ↓
Coverage & Completion
  ↓
Testing
  ↓
Optimisation
  ↓
Final Demonstration

# Final Goal

The completed system should allow the TurtleBot3 to autonomously:

- [ ] Navigate an unknown environment.
- [ ] Build and continuously update a map of the environment.
- [ ] Identify unexplored areas.
- [ ] Select appropriate locations to explore.
- [ ] Navigate between exploration points without manual control.
- [ ] Avoid obstacles and recover from navigation failures.
- [ ] Search the environment for 6×6 ArUco targets.
- [ ] Detect and identify the target markers using the camera.
- [ ] Estimate and record the location of each detected target.
- [ ] Continue exploring after detecting a target.
- [ ] Determine when the search/exploration is complete.
- [ ] Produce a final map showing the explored environment and detected target locations.
- [ ] Demonstrate the complete system reliably from start to finish.
