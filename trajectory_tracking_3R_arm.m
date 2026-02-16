% trajectory_tracking_3R_arm.m
% Jack Adams
% ECE 5463 – Ohio State University
% 3-DOF Planar Robot Arm with Iterative Inverse Kinematics
% and Computed-Torque PD Control for User-Drawn Trajectory Tracking

clc;
clear;
close all;

%% Robot Parameters
L1 = 1; % meters
L2 = 1;
L3 = 1;

m1 = 1; % arbitrary masses
m2 = 1;
m3 = 1;

g = 9.81; % gravity (m/s^2)

workspace = L1 + L2 + L3; % maximum reach when fully extended

%% Workspace Plot
theta = linspace(0,2*pi,500);
x_ws = workspace * cos(theta);
y_ws = workspace * sin(theta);

figure;
hold on;
axis equal;
grid on;

plot(x_ws, y_ws, 'k:');
xlim([-workspace workspace]); 
ylim([-workspace workspace]);
xlabel('X'); 
ylabel('Y');
title('3R Planar Arm Workspace');
scatter(0,0,'k','filled'); % base location

%% User-Drawn Path Input
disp('Click to draw a path. Press ENTER when done.');

[X_raw, Y_raw] = ginput();   % Unlimited clicks, press ENTER to finish

plot(X_raw, Y_raw, 'm--','LineWidth',1.2);  % raw path
scatter(X_raw(1), Y_raw(1), 70, 'g', 'filled'); % starting marker

% Smooth and resample path using spline interpolation
numPoints = 400;
t_raw = linspace(0,1,length(X_raw));
t = linspace(0,1,numPoints);

Xd = interp1(t_raw, X_raw, t, 'spline');
Yd = interp1(t_raw, Y_raw, t, 'spline');

plot(Xd, Yd, 'm','LineWidth',2);
title('User Path Loaded');

%% Inverse Kinematics for Each Path Point
q_path = zeros(3, numPoints);   % joint trajectory storage

for i = 1:numPoints
    q_path(:,i) = IK(Xd(i), Yd(i), L1, L2, L3);
end

%% Trajectory Time Vector
T_total = 12;  % total trajectory duration (seconds)
t_des   = linspace(0, T_total, numPoints);
dt = t_des(2) - t_des(1); % time step

% Compute desired joint velocities and accelerations
% Feedforward terms improve tracking performance
qd_des  = [zeros(3,1), diff(q_path,1,2) / dt];
qdd_des = [zeros(3,1), diff(qd_des,1,2) / dt];

%% PD Computed-Torque Controller

% Gains
Kp = diag([1000 800 600]);   % proportional gains
Kd = diag([120  80  40]);    % derivative gains

% Joint-space inertia matrix (simplified diagonal model)
M = diag([3 2 1]); % Larger inertia for proximal joints

% Joint viscous friction
B = diag([2 1.5 1]); % Helps smooth motion and reduce oscillations

% Allocate storage matrices
q_PD   = zeros(3, numPoints);
qd_PD  = zeros(3, numPoints);
qdd_PD = zeros(3, numPoints);

% Initial conditions
q_PD(:,1)  = q_path(:,1); 
qd_PD(:,1) = [0;0;0];

for k = 1:numPoints-1

    % Position and velocity error
    e    = q_path(:,k)  - q_PD(:,k);
    edot = qd_des(:,k)  - qd_PD(:,k);

    % Gravity torque
    tauG = tau_g(q_PD(:,k), L1, L2, L3, m1, m2, m3, g);

    % Computed-torque control law
    tauPD = M*qdd_des(:,k) + Kp*e + Kd*edot - tauG;

    % Joint dynamics: M*qdd + B*qd = tau
    qdd_PD(:,k) = M \ (tauPD - B * qd_PD(:,k));

    % Integrate
    qd_PD(:,k+1) = qd_PD(:,k) + qdd_PD(:,k)*dt;
    q_PD(:,k+1)  = q_PD(:,k)  + qd_PD(:,k)*dt;
end

%% Tracking Error Analysis

% Desired end-effector path
EE_des = zeros(2, numPoints);
for i = 1:numPoints
    [~,~,~,~,xd,yd] = FK(q_path(:,i), L1, L2, L3);
    EE_des(:,i) = [xd; yd];
end

% Actual end-effector path
EE_act = zeros(2, numPoints);
for i = 1:numPoints
    [~,~,~,~,xa,ya] = FK(q_PD(:,i), L1, L2, L3);
    EE_act(:,i) = [xa; ya];
end

% Euclidean tracking error
EE_err = vecnorm(EE_des - EE_act);

%% Plot Tracking Error
figure;
plot(t_des, EE_err, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Error (m)');
title('End-Effector Tracking Error Over Time');
grid on;

%% Animation
figure(1); hold on;
title('3R Robot Path Animation (PD Controlled)');

h1 = plot([0 0],[0 0],'r','LineWidth',3);
h2 = plot([0 0],[0 0],'g','LineWidth',3);
h3 = plot([0 0],[0 0],'b','LineWidth',3);

for i = 1:numPoints
    q = q_PD(:,i);

    [x1,y1,x2,y2,x3,y3] = FK(q, L1, L2, L3);

    set(h1,'XData',[0 x1],'YData',[0 y1]);
    set(h2,'XData',[x1 x2],'YData',[y1 y2]);
    set(h3,'XData',[x2 x3],'YData',[y2 y3]);

    drawnow limitrate
    pause(0.005)
end

%% Forward Kinematics
function [x1,y1,x2,y2,x3,y3] = FK(q,L1,L2,L3)
    theta1 = q(1); 
    theta2 = q(2); 
    theta3 = q(3);

    x1 = L1*cos(theta1);
    y1 = L1*sin(theta1);

    x2 = x1 + L2*cos(theta1+theta2);
    y2 = y1 + L2*sin(theta1+theta2);

    x3 = x2 + L3*cos(theta1+theta2+theta3);
    y3 = y2 + L3*sin(theta1+theta2+theta3);
end

%% Iterative Inverse Kinematics (Damped Least Squares)
function q = IK(Xd, Yd, L1, L2, L3)

    persistent q_prev
    if isempty(q_prev)
        q_prev = [0; 0; 0];
    end

    % Project target inside reachable workspace
    R = hypot(Xd, Yd);
    maxReach = L1 + L2 + L3;

    if R > maxReach
        scale = maxReach / R;
        Xd = Xd * scale;
        Yd = Yd * scale;
    end

    q = q_prev;
    maxIter = 40;
    tol = 1e-4;
    maxStep = deg2rad(10);

    for k = 1:maxIter
        [~, ~, ~, ~, x3, y3] = FK(q, L1, L2, L3);

        e = [Xd - x3; Yd - y3];

        if norm(e) < tol
            break;
        end

        th1 = q(1); th2 = q(2); th3 = q(3);

        s1 = sin(th1); c1 = cos(th1);
        s12 = sin(th1+th2); c12 = cos(th1+th2);
        s123 = sin(th1+th2+th3); c123 = cos(th1+th2+th3);

        J = [
            -L1*s1 - L2*s12 - L3*s123,  -L2*s12 - L3*s123,  -L3*s123;
             L1*c1 + L2*c12 + L3*c123,   L2*c12 + L3*c123,   L3*c123
        ];

        lambda = 1e-3;
        A = J*J.' + lambda*eye(2);
        dq = J.' * (A \ e);

        dq = max(min(dq, maxStep), -maxStep);
        q = q + dq;
    end

    if any(~isfinite(q))
        q = q_prev;
    end

    q_prev = q;
end

%% Gravity Torque Model
function tau = tau_g(q, L1, L2, L3, m1, m2, m3, g)

    q1 = q(1); q2 = q(2); q3 = q(3);

    Lc1 = L1/2;
    Lc2 = L2/2;
    Lc3 = L3/2;

    tau1 = g*( ...
        m1*Lc1*cos(q1) ...
      + m2*( L1*cos(q1) + Lc2*cos(q1+q2) ) ...
      + m3*( L1*cos(q1) + L2*cos(q1+q2) + Lc3*cos(q1+q2+q3) ) );

    tau2 = g*( ...
        m2*( Lc2*cos(q1+q2) ) ...
      + m3*( L2*cos(q1+q2) + Lc3*cos(q1+q2+q3) ) );

    tau3 = g*( m3*( Lc3*cos(q1+q2+q3) ) );

    tau = [tau1; tau2; tau3];
end
