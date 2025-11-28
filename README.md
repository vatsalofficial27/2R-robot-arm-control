# 2R-robot-arm-control
A simulation of a 2-link robot arm that tracks fixed and moving targets using Jacobian-based control in MATLAB.

File structure
twoR_SerialManipulator.m # Main script
fk2R.m # Forward kinematics function
ik2R.m # Inverse kinematics function

Features:
- Forward kinematics (fk2R)
- Inverse kinematics with reachability check (ik2R)
- Jacobian-based task-space control
- PD control for position regulation
- Damped least-squares inverse for stable velocity mapping
- Point-to-point control
- Moving-target trajectory tracking
- Joint-limit handling and velocity saturation
- Simulation history storage for plotting/analysis
