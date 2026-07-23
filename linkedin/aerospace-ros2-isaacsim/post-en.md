# From Aerospace OLP to ROS 2 + NVIDIA Isaac Sim

Why has aerospace manufacturing relied so heavily on offline programming platforms such as DELMIA—and, in smaller engineering workflows, RoboDK?

Not simply to generate robot code.

Aircraft drilling is a coordinate-system problem before it is a motion-planning problem.

A drilling target must remain geometrically consistent across:

• Aircraft and product coordinates
• Fixture and workpiece coordinates
• Robot base coordinates
• Robot flange and calibrated TCP
• Hole center and surface-normal direction

When positioning requirements are measured in tenths of a millimeter, a robot can be highly repeatable and still miss the CAD target in absolute space.

Errors in base registration, joint zero offsets, TCP calibration, panel location, and frame transformation accumulate at the drill tip. Orientation also matters: the drill must follow the local surface normal, not merely reach an XYZ point.

This is why OLP became important in aerospace manufacturing:

• CAD-to-manufacturing continuity
• Workcell and TCP calibration
• Reach, singularity, and collision validation
• Offline validation without stopping production
• Reusable deterministic process logic

For my latest portfolio project, I recreated this rule-based workflow using ROS 2 and NVIDIA Isaac Sim.

The prototype includes:

• An official six-axis UR10e articulation
• A curved aircraft-panel surrogate with ten DRPE-style bushings
• Explicit base, TCP, joint, and hole frames
• Collision-aware cuMotion
• A deterministic process sequence: Approach → Align → Dock → Clamp → Drill → Verify → Retract
• ROS 2 mission, ACK, status, J1–J6, and TCP topics
• A separate terminal node for closed-loop command and feedback

In the recorded trial, ROS 2 publishes `RUN_HOLE H01`. Isaac Sim accepts it, executes the full sequence, and returns live joint, TCP, process-state, and completion data through Fast DDS.

The key lesson: ROS 2 and Isaac Sim should not be presented as a simple replacement for production OLP.

The stronger architecture preserves OLP’s geometric discipline while adding:

• Open runtime interfaces
• Physics-based digital-twin testing
• Observable, recordable system states
• Automated regression testing
• Synthetic-data generation
• A path toward vision, force control, and adaptive policies

This is a simulation prototype, not a production-qualified drilling system. A real deployment would still require physical TCP identification, workpiece localization, laser-tracker or vision compensation, force/torque feedback, and validation on real material stacks.

Built and iterated with Codex as a development partner.

Where do you see the right boundary between deterministic OLP and adaptive robotics in aerospace manufacturing?

#AerospaceManufacturing #Robotics #ROS2 #IsaacSim #DigitalTwin #OfflineProgramming
