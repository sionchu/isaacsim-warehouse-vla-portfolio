# Aerospace drilling VLA digital twin

## Portfolio objective

This extension demonstrates task-level vision-language-action selection and deterministic DRPE bushing docking on a synthetic aircraft assembly cell. A procedural collaborative robot carries a compact R-eVo-inspired drilling head to ten bushings distributed across two rows on a curved aircraft-skin surrogate.

The project uses bushing docking as a peg-in-hole proxy. It is intended to generate repeatable demonstrations, UI interactions, telemetry, and training examples before adding high-fidelity cutting physics.

## Public industrial references

- [Broetje RACe](https://broetje-automation.de/products/automated-equipment/fastening-systems/product/race/) describes automated positioning, product referencing, clamping, drilling, countersinking, and unclamping on a mobile or stationary robot platform.
- [Electroimpact ADU-Bot](https://www.electroimpact.com/Products/adubot/) describes collaborative insertion of an electric ADU into existing concentric-collet fixtures, tool changing, jig resynchronization, and hole-level torque/speed/position logging.
- [SETI-TEC light automation](https://www.desouttertools.com/en-us/solutions/light-automation) describes the compact R eVo robotic end effector, modular heads, live drilling visualization, and network-ready cycle storage.
- The [Lockheed Martin tooling manual](https://www.lockheedmartin.com/content/dam/lockheed-martin/aero/documents/scm/Quality-Requirements/Control-Specs/tms_mc_015_rev32.pdf) defines the `DRPE` tooling code as a drill plate containing bushings or adapter-size holes for drilling and reaming a hole pattern.

These references informed functional requirements only. No manufacturer CAD, software, confidential process data, or qualification claims are included. Product and company names remain the property of their respective owners.

## Digital-twin layout

```text
/World/AeroDrillVLA
├─ AircraftPanel
│  ├─ Skin                         curved procedural aircraft panel
│  └─ DRPE
│     ├─ UpperPlate
│     ├─ LowerPlate
│     └─ Holes/H01..H10
│        ├─ Bushing
│        ├─ Bore
│        └─ Centerline
├─ Cobot                           procedural kinematic visual surrogate
│  └─ REvoInspiredTool
├─ StatusLights/H01..H10
├─ ProcessCabinet
└─ SafetyZone
```

Each hole owns a center point, outward surface normal, nominal position residual, nominal normal residual, and material stack. The tool TCP approaches along the normal, docks into the DRPE bushing, clamps, executes a drilling surrogate, verifies the cycle, and retracts.

## Control architecture

```text
English/Korean instruction + 2 x 5 visual hole state
                        |
                        v
             trained aero VLA-lite
                 hole selection
                        |
                        v
       H01-H10 sequence/manual safety constraint
                        |
                        v
  Direct Dock / Vision Refine / Spiral Search gate
                        |
                        v
 Approach -> Align -> Search -> Dock -> Clamp -> Drill -> Verify -> Retract
```

The neural policy only selects a task-level hole. It does not command joints, contact forces, or spindle motion. Batch mode deliberately enforces H01 through H10 even if the network predicts a different pending hole. This prevents a learned policy from violating the operator-defined process order.

## Training

```powershell
cd C:\robotics-portfolio
.\scripts\train_aero_vla.ps1
```

The trainer creates synthetic examples containing:

- manual H01-H10 requests;
- next-pending-hole commands;
- upper/lower-row commands;
- lowest-alignment-risk commands;
- completion masks;
- normalized position and normal residual maps.

The checked-in checkpoint was trained on 6,000 synthetic examples for 35 epochs and reached 95.1% generated validation accuracy. The metric measures only synthetic hole-selection classification.

## Run and record

```powershell
cd C:\robotics-portfolio
Set-ExecutionPolicy -Scope Process Bypass
.\scripts\start_aero_drill.ps1
```

Use `Run Selected Hole` for an individual cycle or `Run H01-H10 Batch` for the full sequence. `Toggle Centerlines` shows or hides the green surface-normal guides.

Reproduce the video and dashboard:

```powershell
.\scripts\record_aero_drill.ps1
```

Outputs:

- `recordings/aero_drill_trial.mp4`
- `recordings/aero_drill_trial_thumbnail.png`
- `recordings/aero_drill_events.jsonl` when running interactively

## Verification

```powershell
C:\isaacsim\python.bat tests\isaac_aero_drill_smoke.py
C:\isaacsim\python.bat tests\isaac_aero_extension_smoke.py
```

The controller smoke test creates the USD stage, loads the trained model, executes a manually constrained H03 cycle, and completes every remaining DRPE hole in process order. The extension smoke test additionally enables the packaged UI extension and verifies that it creates `/World/AeroDrillVLA`.

## Current fidelity boundary

- The collaborative robot is a procedural kinematic visual surrogate, not a vendor articulation or validated robot model.
- The R-eVo-inspired tool is generic portfolio geometry and not SETI-TEC CAD.
- Axial force, spindle speed, feed, material stack, and quality scores are synthetic telemetry.
- Docking represents nosepiece/collet alignment with a DRPE bushing; it does not simulate chip formation, drilling heat, burrs, delamination, or true cutting forces.
- The scene is not an aerospace process qualification or production safety assessment.

## Recommended next steps

1. Replace the visual cobot with a calibrated Franka, UR, FANUC CRX, or KUKA articulation.
2. Add PhysX contact and wrist force/torque sensing.
3. Give the DRPE bushings SDF or decomposed collision geometry and validate physical clearance.
4. Estimate hole center and surface normal from a wrist camera or depth sensor.
5. Implement admittance control and measured-force spiral search.
6. Replace synthetic drilling telemetry with a material-removal or experimentally identified process model.
7. Export demonstration episodes with RGB/depth, robot state, force, selected hole, and action labels for a larger VLA policy.
