# Control Chain Evidence

Evidence labels used here:

- `hardware_validated`: verified during earlier Raspbot learning sessions.
- `source_inspected`: confirmed by local source inspection.
- `integration_pending`: designed but not yet connected in this repository.

## Mobile Base Command Path

`hardware_validated`

The Raspbot learning log records the ROS2 command path:

```text
keyboard/custom node
-> /cmd_vel
-> /driver_node
-> motor-driver path
-> wheels
```

The important public contract is `/cmd_vel`, whose message type is
`geometry_msgs/msg/Twist`.

## Camera Path

`hardware_validated`

The learning log records the camera path:

```text
camera
-> /dev/video0
-> OpenCV frame
-> cv_bridge
-> /image_raw
-> subscriber/OpenCV processing
```

## Dynamic Execution Boundary

`source_inspected`

Local Raspbot AI-agent materials include high-level action planning code and
execution code. The project does not copy vendor implementation. It uses the
architectural lesson: a planner-to-executor boundary should be typed,
validated, auditable, and conservative before any robot command is emitted.

## V1 Adapter Boundary

`integration_pending`

This repository keeps the ROS2 backend isolated. The V1 core validates and
replays commands as plain Python. A future adapter can map an approved
`DriveAction` to a ROS2 `geometry_msgs/msg/Twist`.
