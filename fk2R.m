function [x, y] = fk2R(theta1, theta2, L1, L2)
% fk2R: forward kinematics for a planar 2R serial manipulator
% Inputs:
%   theta1, theta2  - joint angles (radians)
%   L1, L2          - link lengths
% Outputs:
%   x, y            - end-effector coordinates

x = L1*cos(theta1) + L2*cos(theta1 + theta2);
y = L1*sin(theta1) + L2*sin(theta1 + theta2);
end