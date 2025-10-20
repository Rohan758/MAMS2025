%% SETUP SCRIPT FOR FLOATING BASE MANIPULATOR

clear; clc;

%% 1. Define Constants
rho_Al = 2700; % Density of Aluminum 6061 in kg/m^3

%% 2. Satellite Body (Hollow Cube)
L_outer = 0.30;   % Outer side length (m)
t_wall = 0.025;  % Wall thickness (m)
L_inner = L_outer - 2 * t_wall; % Inner side length

% Calculate mass (Mass of outer cube - Mass of inner cube)
V_outer = L_outer^3;
V_inner = L_inner^3;
mass_sat = rho_Al * (V_outer - V_inner);
disp(['Mass of satellite: ', num2str(mass_sat), ' kg']);
% Calculate Inertia Tensor (I_outer - I_inner)
I_outer = (1/6) * rho_Al * V_outer * L_outer^2 * eye(3);
I_inner = (1/6) * rho_Al * V_inner * L_inner^2 * eye(3);
I_sat = I_outer - I_inner; % Inertia tensor [Ixx, Iyy, Izz]

%% 3. Manipulator Links (Solid Cylinders)
L_arm = 0.40;     % Length of one link (m)
r_arm = 0.015;    % Radius of link (m)
V_arm = pi * r_arm^2 * L_arm;
mass_arm = rho_Al * V_arm;

% Inertia Tensor for a cylinder rotating about its end
Ixx_arm = (1/12) * mass_arm * (3*r_arm^2 + L_arm^2);
Iyy_arm = Ixx_arm;
Izz_arm = (1/2) * mass_arm * r_arm^2;
I_arm = diag([Ixx_arm, Iyy_arm, Izz_arm]);

%% 4. Reaction Wheels (Solid Disks/Cylinders)
R_rw = 0.10;      % Radius of reaction wheel (m)
t_rw = 0.02;      % Thickness of reaction wheel (m)
V_rw = pi * R_rw^2 * t_rw;
mass_rw = rho_Al * V_rw;

% Inertia Tensor for a disk
Ixx_rw = (1/12) * mass_rw * (3*R_rw^2 + t_rw^2);
Iyy_rw = Ixx_rw;
Izz_rw = (1/2) * mass_rw * R_rw^2; % Along the spinning axis
I_rw = diag([Ixx_rw, Iyy_rw, Izz_rw]);

disp('Parameters loaded into workspace.');