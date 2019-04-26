%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The two-link manipulator with CLF-based controller
% Author: Yang Li
% Tokyo Institue of Technology
% Apr 24, 2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
clc

%% Initialization.m subscript contains the parameters
Initialization;

%% Controller parameters
K_p = 120;  % PD controller P gain
K_d = 20; % PD controller D gain

% rapid exponentially stablizing(RES)
epsi = 0.2; 

n = 2; % number of states
m = 2; % number of inputs
Q = eye(n); % Positive definite Q for CARE

%% Simulation parameters
tstart = 0;
tfinish = 0.5;
dt = 1e-3;

%% Trajectory data
t = tstart:dt:tfinish;

% circular trajectory
AngularVel = pi;    % angular velocity
TrajRadius = 1.45;         % trajectory radius
DesiredTrajectory = TrajRadius .* [cos(AngularVel*t);sin(AngularVel*t)];
dDesiredTrajectory = TrajRadius .* [-AngularVel*sin(AngularVel*t);
    AngularVel*cos(AngularVel*t)];
ddDesiredTrajectory = zeros(2,length(t));

% % fixed point 
% DesiredTrajectory = [0.7*ones(1,length(t));
%     1*ones(1,length(t))];
% dDesiredTrajectory = zeros(2,length(t));
% ddDesiredTrajectory = zeros(2,length(t));

%% Simulation starts!
X = X0;
X_real = X0';
i = 1; % trajectory sequence
eta_out = [];

for t = tstart:dt:tfinish
    
    th1 = X(1);
    th2 = X(2);
    dth1 = X(3);
    dth2 = X(4);    % TODO : Don't want to see them
    
    % Output (position of EOF)
    [joint2, EOF, dEOF] = TwoLinkOutput(X);
    
    % Visualization
    figure(1)
    plot([Ox;joint2(1)],[Oz;joint2(2)],'Linewidth',2) % plot joint 1
    hold on
    plot([joint2(1);EOF(1)],[joint2(2);EOF(2)],'Linewidth',2) % plot joint 2
    plot(DesiredTrajectory(1,i),DesiredTrajectory(2,i),'o','Linewidth',3)
    axis([-1.5 1.5 -1.5 1.5]) % fix the plot window
    hold off
    
    % Manipulator dynamics: M*ddq + H = Tau
    [M,H] = SystemMatrix(X);
    
    % error system: d_eta = f - dr + g * Tau
    eta = [EOF - DesiredTrajectory(:,i);
        dEOF - dDesiredTrajectory(:,i)];
    eta_out = [eta_out , eta]; % debug
    
    f = [ - dth1*(cos(th1 + th2)/2 + cos(th1)) - (dth2*cos(th1 + th2))/2, -(cos(th1 + th2)*(dth1 + dth2))/2;
        - dth1*(sin(th1 + th2)/2 + sin(th1)) - (dth2*sin(th1 + th2))/2, -(sin(th1 + th2)*(dth1 + dth2))/2] * [dth1; dth2] - [ - sin(th1 + th2)/2 - sin(th1), -sin(th1 + th2)/2;
        cos(th1 + th2)/2 + cos(th1),  cos(th1 + th2)/2] * inv(M) * H;
    
    g = [ - sin(th1 + th2)/2 - sin(th1), -sin(th1 + th2)/2;
        cos(th1 + th2)/2 + cos(th1),  cos(th1 + th2)/2] * inv(M);
    
    dot_r = ddDesiredTrajectory(:,i);

    % Input-output linearization
    v = - epsi^-2 * K_p * (eta(1:2)) - epsi^-1 * K_d * (eta(3:4)); % output feedback
    Tau = pinv(g,3e-3) * (-f + v + dot_r); % input (avoid singularity)
    % TODO: Saturation Controller(higher and lower bound!)
    
    % Input
    X = X + [dth1; dth2; inv(M)*(Tau - H)]*dt;
    X_real = [X_real; X'];
    
    i = i + 1;
end

%% QP Controller for CLF-based
% TODO: Code the QP controller!!!


%% Debug plotting

% figure(2) % EOF plotting
% subplot(1,2,1)
% plot(X_real(:,1))
% subplot(1,2,2)
% plot(X_real(:,2))

figure(3) % eta state system plotting
subplot(2,1,1)
hold on
plot(eta_out(1,:))
plot(eta_out(2,:))
hold off
subplot(2,1,2)
hold on
plot(eta_out(3,:))
plot(eta_out(4,:))
hold off