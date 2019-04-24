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
K_p = 10000;  % PD controller P gain
K_d = 200; % PD controller D gain
n = 2; % number of states
m = 2; % number of inputs
Q = eye(n); % Positive definite Q for CARE

%% Simulation parameters
tstart = 0;
tfinish = 1;
dt = 1e-3;

%% Trajectory data
t = tstart:dt:tfinish;
DesiredTrajectory = [cos(7.85397*t);sin(7.85397*t)];
dDesiredTrajectory = [-7.85397*sin(2.5*pi*t);7.85397*cos(2.5*pi*t)];
ddDesiredTrajectory = [-61.6849*cos(7.85397*t);-61.6849*sin(7.85397*t)];

%% Simulation starts!
X = X0;
X_real = X0';
i = 1;

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
    plot(DesiredTrajectory(1,i),DesiredTrajectory(2,i),'o','Linewidth',5)
    axis([-1.5 1.5 -1.5 1.5]) % fix the plot window
    hold off
    
    % Manipulator dynamics: M*ddq + H = Tau
    [M,H] = SystemMatrix(X);
    
    % Input-output linearization (error-based)
    % output system: d_eta = f - dr + g * Tau
    eta = [EOF - DesiredTrajectory(i);
        dEOF - dDesiredTrajectory(i)];
    
    f = [ - dth1*(cos(th1 + th2)/2 + cos(th1)) - (dth2*cos(th1 + th2))/2, -(cos(th1 + th2)*(dth1 + dth2))/2;
        - dth1*(sin(th1 + th2)/2 + sin(th1)) - (dth2*sin(th1 + th2))/2, -(sin(th1 + th2)*(dth1 + dth2))/2] * [dth1; dth2] - [ - sin(th1 + th2)/2 - sin(th1), -sin(th1 + th2)/2;
        cos(th1 + th2)/2 + cos(th1),  cos(th1 + th2)/2] * inv(M) * H;
    % TODO: Singularity???????
    
    g = [ - sin(th1 + th2)/2 - sin(th1), -sin(th1 + th2)/2;
        cos(th1 + th2)/2 + cos(th1),  cos(th1 + th2)/2] * inv(M);
    
    v = -K_p * (eta(1:2)) - K_d * (eta(3:4)); % output feedback
    
    Tau = pinv(g) * (-f + v + ddDesiredTrajectory(i));
    
    
    % Input
    X = X + [dth1; dth2; inv(M)*(Tau - H)]*dt;
    X_real = [X_real; X'];
    
    i = i + 1;
end

figure(2)
subplot(1,2,1)
plot(X_real(:,1))
subplot(1,2,2)
plot(X_real(:,2))