function [M,H] = ActualSystem(X)

global a1 l1 m1 I1 d1 a2 l2 m2 I2 d2
global phi1 phi2 phi3 g
global Ox Oz

% retrive the state
th1 = X(1);
th2 = X(2);
dth1 = X(3);
dth2 = X(4);

% System matrix

% Style 1 : M*ddq + (-phi3*sin(th2)*H + D)*dq + G = Tau
M = [phi1+phi2+2*phi3*cos(th2) phi2+phi3*cos(th2);
    phi2+phi3*cos(th2) phi2];
H = [dth2 dth1+dth2;
    -dth1 0];
D = [d1 0;
    0 d2];
G = g*[m1*l1*cos(th1)+m2*(a1*cos(th1)+l2*cos(th1+th2));
    m2*l2*cos(th1+th2)];

% Style 2 : M*ddq + H = Tau
H = (-phi3 * sin(th2) * H + D) * [dth1;dth2] + G;

end