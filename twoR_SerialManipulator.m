% Demo for 2R serial manipulator: FK, IK, point-to-point control, trajectory tracking
% Usage: run this script in MATLAB. It produces two animations and saves mp4 files.

clear; close all; clc;

%% Parameters
L1 = 1.0;   % m
L2 = 0.7;   % m

% Joint limits (radians)
qmin = [-pi; -pi];
qmax = [ pi;  pi];

% Initial joint angles (rad)
q0 = [deg2rad(30); deg2rad(20)];

%% Visualization settings
make_videos = true;
video_fps = 30;

%% ========== PART A: Point-to-Point Control ==========
% Move end effector from start point to target point using task-space PD 

% Start and target in workspace:
[xs, ys] = fk2R(q0(1), q0(2), L1, L2);
start_pos = [xs; ys];
target_pos = [0.6; 0.9];   % example

% PD-Controller gains 
Kp_task = 5.0;   % proportional on task space (m)
Kd_task = 1.0;   % derivative term in task space

% Damped least squares parameter (for near-singularity)
lambda_dls = 0.1;

% Simulation settings
dt = 0.01;
Tmax = 4.0;
time = 0:dt:Tmax;

% initialize
q = q0;
qdot = [0;0];
history_q = zeros(2,length(time));
history_x = zeros(2,length(time));
history_err = zeros(2,length(time));

% target constant (static)
xd = target_pos;
[theta1, theta2, reachable] = ik2R(target_pos(1),target_pos(2),L1,L2)
if reachable
    for k=1:length(time)
        % forward kinematics
        [x_e, y_e] = fk2R(q(1), q(2), L1, L2);
        x = [x_e; y_e];
        % error 
        e = xd - x;

        % compute Jacobian (2x2)
        J = [-L1*sin(q(1)) - L2*sin(q(1)+q(2)), -L2*sin(q(1)+q(2));
              L1*cos(q(1)) + L2*cos(q(1)+q(2)),  L2*cos(q(1)+q(2))];
        xdot = J*qdot;  
        % desired task-space velocity (PD)
        % Here we use a simple PD in task space to produce desired velocity
        v_des = Kp_task*e + Kd_task*(0-xdot); %Since x_dot_target = 0


        % Damped least squares inverse (Mapping task velocities to joint
        % velocities)
        Jt = J';
        qdot_cmd = (Jt * ((J*Jt + lambda_dls*eye(2)) \ v_des));

        % Joint velocity saturation
        qdot_cmd = max(min(qdot_cmd, 2.0), -2.0);

        % integrate joint velocities
        q = q + qdot_cmd*dt;
        % enforce joint limits
        q = max(min(q, qmax), qmin);
        qdot = qdot_cmd;

        % store history
        history_q(:,k) = q;
        history_x(:,k) = x;
        history_err(:,k) = e;
    end
else
    error('The target cannot be reached');
end

% Visualization and save video
fig1 = figure('Name','Point-to-Point Control','Position',[100 100 700 600]);
ax1 = axes(fig1);
title(ax1, 'Point-to-Point Control');
hold(ax1,'on');
% create video writer
if make_videos
    vw1 = VideoWriter('p2p_demo.mp4','MPEG-4');
    vw1.FrameRate = video_fps;
    open(vw1);
end
for k=1:5:length(time)
    cla(ax1);
    plot_arm(history_q(:,k), L1, L2, ax1);
    % plot target
    plot(ax1, target_pos(1), target_pos(2), 'rx', 'MarkerSize',12, 'LineWidth',2);
    % plot path of end-effector
    plot(ax1, history_x(1,1:k), history_x(2,1:k), '--', 'LineWidth',1);
    drawnow;
    if make_videos
        frame = getframe(fig1);
        writeVideo(vw1, frame);
    end
end
if make_videos, close(vw1); end

%% ========== PART B: Trajectory Tracking (moving point) ==========
% target point moves at constant velocity (vx, vy)
x0_track = [0.2; 0.8];
vx = 0.1;  % m/s
vy = -0.1;  % m/s
Ttrack = 10;
dt = 0.02;
time2 = 0:dt:Ttrack;

% controller gains for task-space velocity control
Kp_t = 8.0;

% start from current q
q = q; 
history_q2 = zeros(2,length(time2));
history_x2 = zeros(2,length(time2));
history_target = zeros(2,length(time2));

for k=1:length(time2)
    t = time2(k);
    xd = x0_track + [vx; vy]*t;
    % current ee position
    [x_e, y_e] = fk2R(q(1), q(2), L1, L2);
    x = [x_e; y_e];
    e = xd - x;
    % desired task velocity (proportional to error + target feedforward velocity)
    v_des = [vx; vy] + Kp_t*e;
    % Jacobian
    J = [-L1*sin(q(1)) - L2*sin(q(1)+q(2)), -L2*sin(q(1)+q(2));
          L1*cos(q(1)) + L2*cos(q(1)+q(2)),  L2*cos(q(1)+q(2))];
    % Damped least squares inverse
    qdot_cmd = J' * ((J*J' + lambda_dls*eye(2)) \ v_des);
    % integrate
    q = q + qdot_cmd*dt;
    % enforce joint limits
    q = max(min(q, qmax), qmin);
    history_q2(:,k) = q;
    history_x2(:,k) = x;
    history_target(:,k) = xd;
end

% Visualization and save video
fig2 = figure('Name','Trajectory Tracking','Position',[200 120 700 600]);
ax2 = axes(fig2);
title(ax2, 'Trajectory Tracking (moving target)');
hold(ax2,'on');
if make_videos
    vw2 = VideoWriter('tracking_demo.mp4','MPEG-4');
    vw2.FrameRate = video_fps;
    open(vw2);
end
for k=1:3:length(time2)
    cla(ax2);
    plot_arm(history_q2(:,k), L1, L2, ax2);
    % plot target and path
    plot(ax2, history_target(1,1:k), history_target(2,1:k), '-r','LineWidth',1);
    plot(ax2, history_x2(1,1:k), history_x2(2,1:k), '--b','LineWidth',1);
    plot(ax2, history_target(1,k), history_target(2,k), 'ro', 'MarkerSize',6,'MarkerFaceColor','r');
    legend(ax2, {'Manipulator','Target path','EE path','Current target'}, 'Location','best');
    drawnow;
    if make_videos
        frame = getframe(fig2);
        writeVideo(vw2, frame);
    end
end
if make_videos, close(vw2); end

fprintf('Demos complete. Videos saved: p2p_demo.mp4 and tracking_demo.mp4 (in current folder).\n');


%% Helper plotting function
function plot_arm(q, L1, L2, ax)
    [x1,y1] = deal(L1*cos(q(1)), L1*sin(q(1)));
    x2 = x1 + L2*cos(q(1)+q(2));
    y2 = y1 + L2*sin(q(1)+q(2));
    plot(ax, [0 x1 x2], [0 y1 y2], '-o', 'LineWidth',2,'MarkerSize',6);
    axis(ax, 'equal');
    ax.XLim = [-1.8 1.8];
    ax.YLim = [-1.2 1.8];
    grid(ax,'on');
end
