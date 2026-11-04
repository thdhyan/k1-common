# k1_interfaces

Custom ROS2 messages, services, and actions for Booster K1 humanoid robot.

## Messages

### JointCommand
Commanded joint targets for K1. Supports all 22 joints or a subset.
- `joint_names`: Array of joint names (must match booster_assets)
- `positions`: Target positions in radians
- `velocities`: Optional velocity feedforward (rad/s)
- `efforts`: Optional effort feedforward (Nm)
- `control_mode`: 0=position, 1=velocity, 2=effort

### LocomotionCommand
High-level velocity command with gait control.
- `twist`: Desired cmd_vel (linear x/y, angular z)
- `gait_mode`: 0=walk, 1=stand, 2=recovery
- `policy_active`: Enable/disable policy execution

### RobotStatus
Robot health and state summary.
- `battery_voltage`: Battery voltage (V)
- `battery_percent`: State of charge (0-100%)
- `foot_contact`: Contact sensors [left, right]
- `fault_code`: 0=ok, non-zero=fault
- `fault_description`: Human-readable fault message
- `control_mode`: 0=idle, 1=stand, 2=locomotion, 3=sdk

### PolicyObs
Raw observation vector fed to locomotion policy (for debugging/logging).
- `obs_vector`: Flattened observation (typically 72 dims × 10-step history = 720 elements)
- `history_len`: History window length
- `obs_dim`: Dimension per time step

## Services

### SetGaitMode
Switch gait mode at runtime.
- Request: `mode` (0=walk, 1=stand, 2=recovery)
- Response: `success` (bool), `message` (string)

---
> **Network note (2026-10-27):** the robotics router (192.168.50.1) has no internet uplink. The laptop shares its eduroam (`wlo1`) internet to the robots by NAT: `enp2s0` = 192.168.50.250/24 is the robots' gateway, with `ip_forward=1` and nft table `ip k1nat` (masquerade out `wlo1`, source 192.168.50.0/24). It is lost on laptop reboot; robots use DNS 8.8.8.8/1.1.1.1 (router DNS is wrong). Re-create steps: `docs/network_robotics_5G_2026-10-26.md` ("Robot internet via laptop").
