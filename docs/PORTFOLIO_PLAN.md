# Portfolio roadmap

Build the portfolio as three small, measurable projects rather than one oversized demo.

## 1. ROS 2 sensor bridge

- One mobile robot in a compact indoor scene
- Publish `/clock`, camera, depth, LiDAR, TF, and odometry
- Visualize data in RViz2
- Deliverables: architecture diagram, topic graph, short demo, measured publish rates

## 2. Autonomous navigation

- Use Nav2 with mapping or a supplied map
- Demonstrate localization, global planning, local obstacle avoidance, and recovery
- Deliverables: reproducible launch command, map, three test routes, success-rate and latency table

## 3. Manipulation

- Use a supported arm and MoveIt 2 for pick-and-place
- Add perception or randomized object poses after the deterministic baseline works
- Deliverables: task state machine, planning-scene explanation, success rate over repeated trials, demo video

## Evidence checklist

Each project should include:

- concise problem statement and system architecture
- pinned environment and one-command launch path
- meaningful metrics rather than only screenshots
- known limitations and a failure example
- a two-minute-or-shorter demo video
- clean source code with no generated assets or secrets committed

