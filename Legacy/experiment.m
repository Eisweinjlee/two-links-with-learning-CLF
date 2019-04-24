function X_real = experiment(X0,tstart,dt,tfinish)

global a1 l1 m1 I1 d1 a2 l2 m2 I2 d2
global phi1 phi2 phi3 g
global Ox Oz

X = X0;
X_real = X0';

for t = tstart:dt:tfinish % t = 0:dt:N
    th1 = X(1);
    th2 = X(2);
    dth1 = X(3);
    dth2 = X(4);
    
    % System matrix
    M = [phi1+phi2+2*phi3*cos(th2) phi2+phi3*cos(th2);
        phi2+phi3*cos(th2) phi2];
    H = [dth2 dth1+dth2;
        -dth1 0];
    D = [d1 0;
        0 d2];
    G = g*[m1*l1*cos(th1)+m2*(a1*cos(th1)+l2*cos(th1+th2));
        m2*l2*cos(th1+th2)];
    
    % Input
    tau1 = 0;
    tau2 = 0;
    Tau = [tau1; tau2];
    
    X = X + [dth1; dth2; inv(M)*(Tau - G - (-phi3*sin(th2)*H + D)*[dth1; dth2])]*dt;
    X_real = [X_real; X'];
    
    % Output
%     joint1 = [Ox;Oz];
    joint2 = [cos(th1);sin(th1)];
    EOF = joint2 + 0.5*[cos(th1+th2);sin(th1+th2)];
    
    % Visualization
    figure(1)
    plot([Ox;joint2(1)],[Oz;joint2(2)],'Linewidth',2) % plot joint 1
    hold on
    plot([joint2(1);EOF(1)],[joint2(2);EOF(2)],'Linewidth',2) % plot joint 2
    axis([-1.5 1.5 -1.5 1.5]) % fix the plot window
    hold off
    
end
end