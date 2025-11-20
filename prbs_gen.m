%% ============================================================
% Unified Input Generation Script for Simscape Experiments
% Generates PRBS, Multisine, Chirp for 2 joint torques
% Produces: PRBS1_data, PRBS2_data, MULTISINE1_data, MULTISINE2_data,
%           CHIRP1_data, CHIRP2_data
%% ============================================================

clc; clear; close all;

%% Experiment parameters
fs = 200;               % sampling frequency (Hz)
Ts = 1/fs;              % sample time
Texp = 120;              % experiment duration (s)
N = Texp * fs;          % number of samples
t = (0:N-1)' * Ts;      % column vector time

tau_max = 0.3;          % scale factor for torque amplitude

%% ============================================================
%% 1) PRBS Signals (piecewise constant ±tau_max)
%% ============================================================

prbs1 = 2*(randi([0 1], N, 1)-0.5);      % ±1
prbs2 = 2*(randi([0 1], N, 1)-0.5);      % uncorrelated ±1

u_prbs1 = tau_max * prbs1;
u_prbs2 = tau_max * prbs2;

PRBS1_data = [t, u_prbs1];
PRBS2_data = [t, u_prbs2];


%% ============================================================
%% 2) Multisine Inputs
%% ============================================================

f_min = 0.2;           % min excitation frequency (Hz)
f_max = 8;             % max excitation frequency (Hz)
n_lines = 40;          % number of sinusoids

freqs = linspace(f_min, f_max, n_lines);

% Joint 1 phases
phases1 = 2*pi*rand(n_lines,1);
% Joint 2 phases (independent)
phases2 = 2*pi*rand(n_lines,1);

u_ms1 = zeros(N,1);
u_ms2 = zeros(N,1);

for k = 1:n_lines
    u_ms1 = u_ms1 + sin(2*pi*freqs(k)*t + phases1(k));
    u_ms2 = u_ms2 + sin(2*pi*freqs(k)*t + phases2(k));
end

% Normalize and scale
u_ms1 = tau_max * u_ms1 / max(abs(u_ms1));
u_ms2 = tau_max * u_ms2 / max(abs(u_ms2));

MULTISINE1_data = [t, u_ms1];
MULTISINE2_data = [t, u_ms2];


%% ============================================================
%% 3) Chirp Inputs (frequency sweep)
%% ============================================================

% Chirp parameters (different for each joint to avoid correlation)
f0_1 = 0.1; f1_1 = 5;    % joint 1 sweep start/end frequencies
f0_2 = 0.2; f1_2 = 6;    % joint 2 slightly different sweep

u_chirp1 = tau_max * chirp(t, f0_1, Texp, f1_1);
u_chirp2 = tau_max * chirp(t, f0_2, Texp, f1_2);

CHIRP1_data = [t, u_chirp1];
CHIRP2_data = [t, u_chirp2];


%% ============================================================
%% Done — variables now in workspace, ready for From-Workspace use
%% ============================================================

disp("All torque signals generated successfully.");
