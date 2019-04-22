function X_real = experiment(X0,tstart,dt,tfinish,DesiredValue,K_p,K_d)

global a1 l1 m1 I1 d1 a2 l2 m2 I2 d2
global phi1 phi2 phi3 g
global Ox Oz

X = X0;
X_real = X0';

% Test
TotalT = length(tstart:dt:tfinish);
th1_d = 2.5 * pi * [tstart:dt:tfinish];
th2_d = DesiredValue(2) * ones(TotalT);
dth1_d = DesiredValue(3) * ones(TotalT);
dth2_d = DesiredValue(4) * ones(TotalT);
i=1;

for t = tstart:dt:tfinish % t = 0:dt:N
    th1 = X(1);
    th2 = X(2);
    dth1 = X(3);
    dth2 = X(4);
    
    % System matrix
    % M*ddq + (-phi3*sin(th2)*H + D)*dq + G = Tau
    [M,H] = ActualSystem(X);
    
    % Controller Input
    tau1 = - K_p * (th1 - th1_d(i)) - K_d * (dth1 - dth1_d(i));
    tau2 = - K_p * (th2 - th2_d(i)) - K_d * (dth2 - dth2_d(i));
    Tau = 1000 * [tau1; tau2];
    i = i + 1;
    
%     X = X + [dth1; dth2; inv(M)*(Tau - G - (-phi3*sin(th2)*H + D)*[dth1; dth2])]*dt;
    X = X + [dth1; dth2; inv(M)*(Tau - H)]*dt;
    X_real = [X_real; X'];
    
    % Output
%     joint1 = [Ox;Oz];
    joint2 = [cos(th1);sin(th1)];
    EOF = joint2 + 0.5*[cos(th1+th2);sin(th1+th2)];
    
    EOF_d = [cos(th1_d(i));sin(th1_d(i))]+ ...
    0.5*[cos(th1_d(i)+th2_d(i));sin(th1_d(i)+th2_d(i))];
    
    % Visualization
    figure(1)
    plot([Ox;joint2(1)],[Oz;joint2(2)],'Linewidth',2) % plot joint 1
    hold on
    plot([joint2(1);EOF(1)],[joint2(2);EOF(2)],'Linewidth',2) % plot joint 2
    plot(EOF_d(1),EOF_d(2),'o','Linewidth',3) % The desired configuration
    axis([-1.5 1.5 -1.5 1.5]) % fix the plot window
    hold off
end
    figure(2)
    subplot(1,2,1)
    plot(X_real(:,1))
    subplot(1,2,2)
    plot(X_real(:,2))
end