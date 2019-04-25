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

%% Control design parameters
K_p = 120;  % PD controller P gain
K_d = 20; % PD controller D gain
epsi = 0.4; % Rapidly exponentially stabilizing
n = 2; % number of states
m = 2; % number of inputs
Q = eye(n); % Positive definite Q for CARE

%% Simulation parameters
tstart = 0;
tfinish = 0.5;
dt = 1e-3;

%% Trajectory data
t = tstart:dt:tfinish;
DesiredTrajectory = [cos(t);sin(t)];
dDesiredTrajectory = [-sin(t);cos(t)];
ddDesiredTrajectory = zeros(2,length(t));

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
    dth2 = X(4);    %% TODO : Don't want to see them
    
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
    
    % Input-output linearization (error-based)
    % output system: d_eta = f - dr + g * Tau
    eta = [EOF(1) - DesiredTrajectory(1,i);
        EOF(2) - DesiredTrajectory(2,i);
        dEOF(1) - dDesiredTrajectory(1,i);
        dEOF(2) - dDesiredTrajectory(2,i)];
    eta_out = [eta_out , eta]; % debug
    
    f = [ - dth1*(cos(th1 + th2)/2 + cos(th1)) - (dth2*cos(th1 + th2))/2, -(cos(th1 + th2)*(dth1 + dth2))/2;
        - dth1*(sin(th1 + th2)/2 + sin(th1)) - (dth2*sin(th1 + th2))/2, -(sin(th1 + th2)*(dth1 + dth2))/2] * [dth1; dth2] - [ - sin(th1 + th2)/2 - sin(th1), -sin(th1 + th2)/2;
        cos(th1 + th2)/2 + cos(th1),  cos(th1 + th2)/2] * inv(M) * H;
    % TODO: Singularity??????? BUG in the equation coding, Check later!
    % TODO: the input needs the matrix B?
    
    g = [ - sin(th1 + th2)/2 - sin(th1), -sin(th1 + th2)/2;
        cos(th1 + th2)/2 + cos(th1),  cos(th1 + th2)/2] * inv(M);

    
    v = - epsi^-2 * K_p * (eta(1:2)) - epsi^-1 * K_d * (eta(3:4)); % output feedback
    
    Tau = pinv(g,3e-3) * (-f + v + ddDesiredTrajectory(i));
    
    % Input
    X = X + [dth1; dth2; inv(M)*(Tau - H)]*dt;
    X_real = [X_real; X'];
    
    i = i + 1;
end

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