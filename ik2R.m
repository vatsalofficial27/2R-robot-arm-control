function [theta1_solutions, theta2_solutions, reachable] = ik2R(x, y, L1, L2)
% ik2R: inverse kinematics for planar 2R manipulator
% Returns both elbow-up and elbow-down solutions if reachable.
% Inputs: x,y,L1,L2
% Outputs:
%   theta1_solutions: 2x1 vector (sol1, sol2) in radians
%   theta2_solutions: 2x1 vector (sol1, sol2) in radians
%   reachable: boolean
%
% Uses standard cosine law; numerical safety margin included.

r2 = x^2 + y^2;
cos_theta2 = (r2 - L1^2 - L2^2) / (2*L1*L2);

% reachability check with tolerance
tol = 1e-9;
if cos_theta2 > 1+tol || cos_theta2 < -1-tol
    reachable = false;
    theta1_solutions = [];
    theta2_solutions = [];
    return;
end
reachable = true;
cos_theta2 = min(max(cos_theta2, -1), 1);
theta2_a = acos(cos_theta2);
theta2_b = -theta2_a;

% compute theta1 for each theta2
k1 = L1 + L2*cos(theta2_a);
k2 = L2*sin(theta2_a);
theta1_a = atan2(y, x) - atan2(k2, k1);

k1b = L1 + L2*cos(theta2_b);
k2b = L2*sin(theta2_b);
theta1_b = atan2(y, x) - atan2(k2b, k1b);

theta1_solutions = [wrapToPi(theta1_a); wrapToPi(theta1_b)];
theta2_solutions = [wrapToPi(theta2_a); wrapToPi(theta2_b)];
end
