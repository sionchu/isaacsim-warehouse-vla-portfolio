# UR10e aerospace drilling VLA digital twin

## Portfolio objective

This Isaac Sim 6.0 extension demonstrates task-level vision-language-action selection and deterministic DRPE bushing docking on an aircraft assembly cell. NVIDIA's official UR10e USD articulation carries a generic R-eVo-inspired drilling-head visualization toward ten 10 mm bushing centers on a curved aircraft-skin and drill-plate assembly.

The project uses bushing docking as a peg-in-hole proxy. It produces reproducible UI interactions, six-axis telemetry, coordinate-frame visualization, collision-aware motion, process logs, and video demonstrations before high-fidelity cutting physics is added.

## Public industrial references

- [Broetje RACe](https://broetje-automation.de/products/automated-equipment/fastening-systems/product/race/) describes automated positioning, product referencing, clamping, drilling, countersinking, and unclamping on mobile or stationary robot platforms.
- [Electroimpact ADU-Bot](https://www.electroimpact.com/Products/adubot/) describes collaborative insertion of an electric ADU into concentric-collet fixtures, jig resynchronization, and hole-level process logging.
- [SETI-TEC light automation](https://www.desouttertools.com/en-us/solutions/light-automation) describes the compact R eVo robotic end effector, modular heads, drilling visualization, and network-ready cycle storage.
- The [Lockheed Martin tooling manual](https://www.lockheedmartin.com/content/dam/lockheed-martin/aero/documents/scm/Quality-Requirements/Control-Specs/tms_mc_015_rev32.pdf) defines `DRPE` as a drill plate containing bushings or adapter-size holes for drilling and reaming a hole pattern.

These references informed functional requirements only. The repository contains no manufacturer CAD, confidential process data, or qualification claims.

## Digital-twin layout

```text
/World/AeroDrillVLA
|-- UR10e                              NVIDIA official USD articulation
|   |-- joints                        six PhysicsRevoluteJoint prims
|   |-- base_link
|   |-- shoulder_link
|   |-- upper_arm_link
|   |-- forearm_link
|   |-- wrist_1_link
|   |-- wrist_2_link
|   `-- wrist_3_link
|-- AeroDrillTool                     generic FK-following visual attachment
|   `-- TCP                           local tool coordinate-frame visualization
|-- AircraftPanel
|   |-- FuselageSkin                  curved collision mesh
|   |-- frames, stringers, rivets
|   `-- DRPE
|       |-- UpperPlate
|       |-- LowerPlate
|       `-- Holes/H01..H10
|           |-- Bushing               28 mm outside diameter
|           |-- Bore                  10 mm pilot diameter
|           `-- Centerline
|-- CoordinateFrames
|   |-- UR_Base
|   `-- ActiveHole
|-- StatusLights/H01..H10
|-- ProcessCabinet
`-- SafetyZone
```

The panel is approximately 1.7 m wide and 1.0 m high with a 2.4 m curvature radius. Hole spacing is 180 mm across each row, row spacing is 280 mm, and each bushing is rendered near a realistic drill-plate scale rather than at robot-joint scale.

## Frames and six-axis state

The UI and viewport use these frame conventions:

- `W`: Isaac Sim world frame, Z up.
- `B`: UR10e base frame. The current cell places `B` coincident with `W`.
- `TCP`: drilling-head tip. TCP local `+Z` is the drilling direction toward the panel.
- `H01..H10`: hole center frame. Hole local `+Z` is the outward surface normal.

The panel lists all six actuated joints:

1. J1 `shoulder_pan_joint`
2. J2 `shoulder_lift_joint`
3. J3 `elbow_joint`
4. J4 `wrist_1_joint`
5. J5 `wrist_2_joint`
6. J6 `wrist_3_joint`

Each row displays position in degrees, velocity in degrees per second, and the limits read from the live articulation. `Toggle Frames` controls the base, TCP, active-hole, and link-frame gizmos. `Toggle Colliders` switches Isaac Sim physics-collider visualization.

## Motion and collision architecture

```text
English/Korean instruction + 2 x 5 hole state
                         |
                         v
              trained aero VLA-lite
                  hole selection
                         |
                         v
        H01-H10/manual process-order safety gate
                         |
                         v
       Direct / Vision Refine / Spiral Search
                         |
                         v
  task-space TCP target + surface-normal orientation
                         |
                         v
  NVIDIA cuMotion RMPflow + UR10 collision spheres
                         |
                         v
  J1-J6 position targets on the PhysX articulation
```

The high-level mission phases are:

```text
Approach -> Align -> optional Search -> Dock -> Clamp
         -> Drill -> Verify -> Retract
```

cuMotion uses the official articulation joint state and the supported UR10 robot description to generate fixed-link joint targets. A world binding tracks collision-enabled panel, DRPE, structure, floor, pedestal, and cabinet geometry. The current scene registers 20 static collision shapes and retains the robot description's self-collision model.

The generic drilling-head mesh is updated from cuMotion forward kinematics at `tool0`. It is a visual TCP attachment, not SETI-TEC CAD and not yet part of the UR10 collision-sphere description. The robot links and their collisions remain physical.

## VLA-lite training

```powershell
cd C:\robotics-portfolio
.\scripts\train_aero_vla.ps1
```

The trainer creates synthetic examples containing manual H01-H10 requests, next-pending commands, upper/lower-row commands, lowest-risk commands, completion masks, and normalized position/normal residual maps.

The checked-in checkpoint was trained on 6,000 synthetic examples for 35 epochs and reached 95.1% generated validation accuracy. This metric measures task-level hole classification only. The neural policy does not command joints, contact force, or spindle motion.

## Run and record

```powershell
cd C:\robotics-portfolio
Set-ExecutionPolicy -Scope Process Bypass
.\scripts\start_aero_drill.ps1
```

Use:

- `Run Selected Hole` for one H01-H10 cycle.
- `Run H01-H10 Batch` for strict process order.
- `Toggle Centerlines` for surface-normal guides.
- `Toggle Frames` for B/TCP/hole/link coordinate frames.
- `Toggle Colliders` for physics collision visualization.

Record a compact validated clip or the full batch:

```powershell
.\scripts\record_aero_drill.ps1 -MaxHoles 1
.\scripts\record_aero_drill.ps1 -MaxHoles 10
```

Outputs:

- `recordings/aero_drill_trial.mp4`
- `recordings/aero_drill_trial_thumbnail.png`
- `recordings/aero_drill_events.jsonl` during interactive execution

## Verification

```powershell
C:\isaacsim\python.bat tests\isaac_aero_drill_smoke.py
C:\isaacsim\python.bat tests\isaac_aero_extension_smoke.py
C:\isaacsim\python.bat tests\isaac_aero_ur10e_motion_smoke.py
C:\isaacsim\python.bat tests\isaac_aero_ur10e_mission_smoke.py
```

The checks validate:

- ten-hole logical process ordering and trained VLA-lite loading;
- official UR10e reference loading and six revolute joints;
- absence of the previous procedural `Cobot/Link` geometry;
- coordinate-frame and TCP prim creation;
- at least ten physical collision shapes;
- a collision-aware H03 task-space convergence test;
- a complete H01 cycle without a motion-timeout transition.

The validated H01 mission completed in 358 physics frames (5.97 simulated seconds). Final TCP tracking error was 27.1 mm after retract; near-contact phase errors reduced through Dock, Clamp, Drill, and Verify to approximately 35.0, 7.4, 4.3, and 2.1 mm respectively.

## Current fidelity boundary

- The robot links, joints, limits, drives, and collision geometry come from NVIDIA's official UR10e asset.
- The R-eVo-inspired drilling head is generic portfolio geometry driven by `tool0` forward kinematics.
- The drilling-head mesh is not yet included in the cuMotion collision-sphere model.
- Axial force, spindle speed, feed, material stack, and quality scores are synthetic telemetry.
- Docking represents nosepiece/collet alignment with a DRPE bushing; it does not simulate chip formation, heat, burrs, delamination, or calibrated cutting forces.
- This is not an aerospace process qualification, payload study, or production safety assessment.

## Recommended next steps

1. Add an XRDF tool-link collision model for the drilling head.
2. Replace the visual bore with clearance-valid SDF or decomposed bushing geometry.
3. Add wrist force/torque sensing and measured-force admittance control.
4. Estimate hole center and surface normal from a wrist RGB-D camera.
5. Add joint torque, contact impulse, and minimum robot-to-panel distance logging.
6. Calibrate the UR10e base/TCP against physical cell measurements.
7. Export RGB/depth, J1-J6 state, selected hole, force, and action labels for larger VLA training.
