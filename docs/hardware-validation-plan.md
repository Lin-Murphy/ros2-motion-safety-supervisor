# Hardware Validation Plan

The core project is designed so hardware-facing validation can be added later
without changing the typed action contract.

## Step 1: ROS2 Adapter

Map approved `DriveAction` values to `geometry_msgs/msg/Twist`:

```text
DriveAction(vx, vy, wz, duration_s)
-> Twist.linear.x, Twist.linear.y, Twist.angular.z
```

## Step 2: Dry Run Against ROS2 Graph

Validate:

- `/cmd_vel` type
- publisher/subscriber count
- stop command after every terminal path
- no dependency on `/odom`, `/imu`, `/joint_states`, or full TF unless explicitly added

## Step 3: Low-Speed Controlled Run

Use conservative velocity and duration limits. Record:

- command plan
- emitted `/cmd_vel`
- observed behavior
- stop behavior
- any mismatch between predicted and observed result

## Step 4: Recalibration

Only after controlled runs should normalized V1 command values be calibrated to
physical velocity units or braking-distance estimates.
