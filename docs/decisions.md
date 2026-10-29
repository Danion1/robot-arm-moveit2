# Engineering decisions

Each entry: what was decided, what was considered instead, and why. Values marked **ASSUMED** have no datasheet or test behind them yet; each lives in a single named parameter at the top of its script, to be replaced after a test print.

## Architecture and components

| Decision | Alternatives considered | Why |
|---|---|---|
| Serial arm | Parallelogram (motors near the base, gripper self-levelling) | The closed loop needs mimic joints, is poorly described in URDF and complicates MoveIt2. Accepted more shoulder torque (less payload with the same servos) for simpler software. |
| DYNAMIXEL XM430-W350-T (base, shoulder, elbow), XC430-W240-T (wrist, gripper) | XW430 (product page says 2.3 N·m, ROBOTIS comparison table says 6.9 N·m: not trustworthy); XM540 (heavier, unnecessary) | Chosen by torque calculation from the CAD masses, with the ROBOTIS rule of thumb: load up to 1/5 of stall torque. |
| Reach 278.5 mm (L2 105, L3 90, L4 83.5 mm) | 300 mm | 300 mm was a number from an earlier prototype, not a requirement. Less lever arm, more payload. |
| PLA, printed | Copying plate thickness from aluminium reference arms | Aluminium is ~30× stiffer than PLA; thickness comes from a bending/stress calculation with the PLA datasheet values. |
| Base must be clamped to the table (3× M3) | Free-standing tripod | From the CAD masses, the centre of mass sits 85 mm from the axis versus the tripod edge at 48.5 mm: free, it tips over even without load. |

## CAD

| Decision | Alternatives considered | Why |
|---|---|---|
| CadQuery (code CAD), one script per part, a single owner for every interface dimension | One monolithic script; independent scripts with copied numbers; GUI CAD | A monolith grows unreadable and every change re-runs every check. Copied numbers drift silently (change a clearance in the fork and the link no longer fits, with no error). Each part script runs and checks on its own; only `montagem.py` knows all parts. |
| Detail comes from rules with a source (datasheet, ISO table, calculation, test), not typed numbers | Modelling dimensions by eye | Requirements go in (link lengths, load), geometry comes out, and every number can be traced. |
| ISO screws modelled from the ISO tables inside the project (ISO 4762, ISO 273, ISO 4032) | cq-warehouse fasteners | Unmaintained external dependency. |
| Hex nuts embedded in the plastic | Threading directly into PLA; brass heat-set inserts | Direct threads wear with every reassembly. Inserts are a planned improvement (they need the insert's own hole datasheet). |
| Design load for the forks = servo **stall** torque (4.1 N·m, XM430), safety factor ≥ 3 (ASSUMED) on PLA yield | Sizing for operating load (~1/5 of stall, 0.8 N·m) | Anything the servo itself can do (arm jammed on the table, trajectory error), the plate survives; above stall the servo firmware cuts torque (Overload error, on by default). The plate is never the weakest part in the load path. |
| Horn plate 28.5 × 4.0 mm, 8× M2×5 | 4 screws (fail in bearing); 24 mm width as in ROBOTIS aluminium brackets (never passes up to 6 mm) | Lightest option that passes (`cad/resistencia_forquilha.py`): in-plane bending stress falls with the **square** of the width and only linearly with thickness. The margin is thin (SF 3.17), so `junta.py` repeats the calculation on every run. |
| Fork made of flat plates bolted to a bridge | One-piece printed U | A plate printed flat carries the load in the plane of the layers and the M2 holes come out vertical and round. Cost: more parts and screws. |
| Fork bridge radius from the official STEP (chamfered corners, max radius 16.99 mm) + 0.5 mm clearance (ASSUMED) | Datasheet bounding box (sharp corner, 18.16 mm) | The datasheet box is only an envelope; treating it as geometry would be using a simplification as data. Servo revision risk is covered by a SHA-256 check of the STEP. |
| Link and fork are one piece; the link holds the next servo by a flange on its long end, 2 rows of screws | Separate link plate bolted to the fork; 1 row of screws | Every bolted joint in series with the torque is a weak point. With 1 row the hole in the plate had SF 1.96; with 2 rows, 4.33. |
| Base and tower: vertical base servo in a cradle, tower is one L-shaped part printed on its side | Tower of side plates + blocks (SF 2.3); tower printed upright (layers loaded across) | The shoulder moment bends the L in the plane of the layers. |
| Shoulder raised 14 mm | Lower shoulder | Without it the arm bridge hit the tower plate at 12° down. Requirement: going down, the arm is stopped by the table, never by its own base. |
| Parallel-jaw gripper, rack and pinion (m 2, z 18, 8 mm face) | Parallelogram fingers (closed loop: URDF cannot represent it, the simulation would fake the motion); one fixed + one moving finger (closes in an arc); geared fingers (printed gear backlash) | In URDF each jaw is a prismatic joint with a mimic, so the simulation is exactly the real mechanism. m 2, z 18 is the smallest pinion that passes Lewis bending at XC430 stall with SF 3 (SF 3.36); z 18 is the minimum without undercut against a rack. |

## Simulation and control

| Decision | Alternatives considered | Why |
|---|---|---|
| URDF generated by a script from the CAD (masses, inertias, meshes, limits) | URDF written by hand | One source of truth; the generator also checks the hand-written fragment (`world` + `base_link`). |
| Collision geometry = one box per part, 1 mm margin (ASSUMED) | Detailed mesh (slow); convex hull per link | A convex hull closes the gripper and fork openings. The same boxes feed the URDF, the MoveIt2 collision matrix and the Gazebo pick and place world. |
| Analytic IK (planar arm + base yaw, law of cosines); MoveIt2 receives joint goals | MoveIt2's KDL solver; TRAC-IK | With 4 joints KDL solves position only and the gripper orientation ends up arbitrary. The analytic solution takes 4 numbers (x, y, z, gripper pitch) and is checked against the CAD solids. |
| Kinematics lives in the ROS 2 node package (`braco2_nos/cinematica.py`) | In `cad/` | The node imports it directly; the CAD check imports it from the package instead of copying it. |
| ros2_control **position** command interface | Effort command + own PID | Position is the real servos' default mode (internal PID), and the XC430 has no torque mode at all: simulating effort control would not transfer to hardware. Since position control can hide gravity in simulation, the checks read each joint's effort and compare it with the CAD. |
| MoveIt2 with mock hardware first, then Gazebo | Gazebo only; mock only | With mock hardware MoveIt2 is configured and checked against the CAD on its own: if something fails, it can only be MoveIt2. Then only the hardware plugin changes. |
| MoveIt2 config generated by script (SRDF, collision matrix, limits, controllers) | — | Same reason as the URDF; the collision matrix disables pairs that never collided in 10,000 random poses. |
| Hybrid grasp: physics closes the gripper, the node checks that it stopped before its target, then a `DetachableJoint` holds the object | Pure friction (unstable in Gazebo); always attach (hides a bad grasp); teleport (proves nothing) | Best balance between verifying the grasp and working every time. What it does **not** simulate: the object slipping in the gripper while carried (stated in the README). |
| Bullet-Featherstone physics in the pick and place world | DART (Gazebo default); commanding both jaws through a controller | DART does not support mimic joints, so only one jaw moved. Commanding both jaws would not match the real single-pinion gripper. |

## Fabrication assumptions to replace after a test print

All in one parameter each, marked ASSUMED in the scripts: hole and nut clearances (printer ~0.3 mm + hole play 0.2 mm), gear flank clearance 0.1 mm, gripper rail clearance 0.2 mm, minimum wall 1.2 mm (3 perimeters, 0.4 mm nozzle), safety factor 3 on solid-PLA datasheet values (printed parts are weaker and lighter).
