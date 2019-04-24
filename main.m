%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The Episodic Learning with CLF for Uncertainty
% Author: Yang Li
% Tokyo Institue of Technology
% Apr 19, 2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
clc

%% Initialization
% Initialization.m subscript contains the parameters
Initialization;

%% Control design parameters
K_p = 25;  % PD controller P gain
K_d = 1.5; % PD controller D gain
n = 2; % number of states
m = 2; % number of inputs
Q = eye(n); % Positive definite Q for CARE

%% Simulation parameters
tstart = 0;
tfinish = 1;
dt = 1e-3;
% N = 5000; % numbers of time steps

%% Lagrangian system matrix
% [M,H,D,G] = ActualSystem(X);
% [M_hat, H_hat, D_hat, G_hat] = EstimatedSystem(X); 
% % Uncertainty is not added yet!!!!

%% Trajectory data
% make a trajectory data w.r.t. the time
% error = output(EOF) - traj
DesiredValue = [0 pi/4 2.5*pi 0];

%% Simulation
X_real = experiment(X0,tstart,dt,tfinish,DesiredValue,K_p,K_d);    % Nominal system
% Estimated system

%% System and control definitions
% the controller we will use in this experiment
% PD controller, QP controller, Leaning-based controller

%% Input
% controller

%% Learning loop
% please study!!!


%% Run experiments in simulation
% for each controller we have
% simulation handler.m function!!

%% Visualization



