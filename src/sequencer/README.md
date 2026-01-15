# Sequencer Package

SMACH state machine for orchestrating pick actions.

## SMACH States

1. **DETECT_SEGMENT** - Detect and segment object to get 2D coordinates
2. **VALIDATE_GRASP** - Validate if grasp is collision-free
3. **PICK** - Execute pick action

## State Machine Flow

```
DETECT_SEGMENT → VALIDATE_GRASP → PICK
      ↓                ↓              ↓
   failure         invalid         failure
```

## Usage

### Run sequencer:
```bash
ros2 run sequencer pick_sequencer
```

### View state machine (in another terminal):
```bash
ros2 run smach_viewer smach_viewer.py
```
Then connect to `/SM_PICK`

## Userdata Flow

- **centroid_2d** (Point) - 2D pixel coordinates from DETECT_SEGMENT
- **depth** (float) - Depth value from DETECT_SEGMENT
- **world_coords** (Point) - 3D world coordinates from VALIDATE_GRASP

## TODO

- [ ] Implement actual service calls in states
- [ ] Add error handling and retry logic
- [ ] Add gripper control integration
- [ ] Implement segmentation service call
- [ ] Implement world_coords service call
- [ ] Implement validate_grasp service call
- [ ] Implement execute_grasp action call
