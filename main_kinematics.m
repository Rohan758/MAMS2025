%% MAIN SCRIPT: FLOATING MANIPULATOR KINEMATICS

clear; clc;

% Define symbolic variables for the system state
% --- Satellite State ---
syms x y z real                     % Position of satellite CoM
syms phi theta psi real             % Orientation (Euler angles ZYX)
syms vx vy vz real                  % Linear velocity of satellite CoM
syms wx wy wz real                  % Angular velocity of satellite CoM

% --- Manipulator State ---
syms q1 q2 real                     % Joint angles
syms dq1 dq2 real                   % Joint angular velocities

% --- Geometric Parameters ---
syms L1 L2 real                     % Length of links (using L1, L2 for clarity)
L1 = 0.4; % Length of link 1
L2 = 0.4; % Length of link 2

%% Step 1: Define the state vectors
X_sat = [x; y; z; phi; theta; psi];      % Satellite pose
V_sat = [vx; vy; vz; wx; wy; wz];      % Satellite twist (linear & angular vel)

q = [q1; q2];                         % Joint angles
dq = [dq1; dq2];                      % Joint velocities

%% Step 2: Get Transformation Matrices using DH Parameters
% T_0_1 is the transform from the satellite base (frame 0) to link 1 (frame 1)
% T_1_2 is the transform from link 1 to link 2
[T_0_1, T_1_2] = get_dh_transforms(q, [L1, L2]);

% The full transform from the satellite base to the end-effector (frame 2)
T_0_2 = T_0_1 * T_1_2;

disp('Transformation from Base to Link 1 (T_0_1):');
pretty(T_0_1)

disp('Transformation from Base to End-Effector (T_0_2):');
pretty(T_0_2)

%% Step 3: Calculate Jacobians
% The Jacobian relates joint velocities to the end-effector's linear and
% angular velocity *relative to the satellite base*.
J = get_arm_jacobian(T_0_1, T_0_2);

disp('Arm Jacobian (J):');
pretty(J)

%% Step 4: Calculate Velocities in the World Frame
% First, get the velocity of the end-effector *relative* to the base
V_e_rel = J * dq; % [vx_rel; vy_rel; vz_rel; wx_rel; wy_rel; wz_rel]

% Now, let's find the total velocity in the world frame. We need the
% rotation matrix from the satellite body frame to the world frame.
% The eul2rotm function doesn't support symbolic variables, so we build
% the ZYX rotation matrix manually.
% R_world_sat = Rz(psi) * Ry(theta) * Rx(phi)

Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];

R_world_sat = Rz * Ry * Rx;

% The velocity of the end-effector in the world frame is:
% V_world = V_satellite + R_world_sat * V_e_rel
% Note: This is a simplified view. The full velocity composition also
% includes the cross product of the satellite's angular velocity with the
% position vector from the satellite's CoM to the end-effector.

% Let's find the position of the end-effector relative to satellite CoM
p_e_rel = T_0_2(1:3, 4); % Position vector from arm base to end-effector

% Total linear velocity of the end-effector in the world frame
v_end_effector_world = V_sat(1:3) + R_world_sat * V_e_rel(1:3) + ...
                       cross(V_sat(4:6), R_world_sat * p_e_rel);

% Total angular velocity of the end-effector in the world frame
w_end_effector_world = V_sat(4:6) + R_world_sat * V_e_rel(4:6);


disp('Total Linear Velocity of End-Effector (World Frame):');
pretty(v_end_effector_world);

disp('Total Angular Velocity of End-Effector (World Frame):');
pretty(w_end_effector_world);