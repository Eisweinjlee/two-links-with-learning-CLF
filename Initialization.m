% Initialzation.m
% The parameters are set in this file for symplicity of main
% symbolic and global values are also defined here

global a1 l1 m1 I1 d1 a2 l2 m2 I2 d2
global phi1 phi2 phi3 g
global Ox Oz

% parameters of links
a1=0.5;     % position of mass along link[m]
l1=1.0;     % length of link 1 [m]
m1=10;      % mass of link 1 [kg]
I1=5;       % Inertia of link 1 [kg.m2]
d1=0;

a2=0.25;    % position of mass along link[m]
l2=0.5;     % length of link 2 [m]
m2=5;       % mass of link 2 [kg]
I2=0.5;     % Inertia of link 2 [kg.m2]
d2=0;

% System matrix 
phi1 = m1*l1^2 + m2*a1^2 + I1;
phi2 = m2*l2^2 + I2;
phi3 = m2*a1*l2;

Ox=0;                   % x of the origin
Oz=0;                   % z of the origin
g=9.81;                 % gravity

% initial configuration
th1 = -pi/4;
th2 = -pi/4;
dth1 = 0.0;
dth2 = 0.0;
X0 = [th1; th2; dth1; dth2];