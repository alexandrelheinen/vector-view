# Modern grasp controller

The release demo runs on **Gazebo Harmonic** with a native controller that closes
the loop on **live contact feedback**. It does not require CoDyCo, YARP, legacy
recordings, or golden trajectory replay.

## Architecture

```text
Phase state machine (STAND → REACH → PRESS → HOLD → RELEASE → RETURN)
        ↓
JointPositionController command topics (/grasp_demo/*)
        ↓
Gazebo physics + VectorView contact sensors
        ↓
contact_monitor.py (upper-box contact feedback during PRESS/HOLD)
```

## Control law

| Phase | Behavior |
| --- | --- |
| STAND / RETURN | Hold the default standing arm posture |
| REACH | Interpolate to the reach posture beside the box |
| PRESS | Increment joint commands until **both** hands touch `object::main::collision` |
| HOLD | Maintain the loaded press posture while contacts stay active |
| RELEASE | Interpolate back toward standing |

PRESS is the task-space replacement for CoDyCo `StageTestTasks`: the goal is
**bilateral upper-box contact**, verified from real `/vectorview/...` messages,
not from a replayed joint log.

## Run

```bash
# Simulation must already be running:
python3 scripts/grasp_task_controller.py

# Wrapper used by the demo scripts:
scripts/animate_grasp.sh
```

## Capture verification

`scripts/capture_comparison.sh` runs this controller and checks:

- `controller=grasp_task_controller`
- `phase=HOLD`
- `both_on_box=true` in the motion snapshot
- contacts, arrows, and plots (V/S/D requirements)

## Replacing deprecated pieces

| Deprecated (2015) | Modern replacement |
| --- | --- |
| CoDyCo `ISIRWholeBodyController` | `grasp_task_controller.py` |
| gazebo-yarp-plugins controlboard | Harmonic `JointPositionController` |
| Recorded joint trajectories | Contact-closed-loop press |

Future work can swap the press increments for Cartesian IK (e.g. ocra-core) once
a reliable world-frame hand pose source exists on Harmonic. The capture contract
stays the same because it is anchored on real contacts and forces.
