function [M,H,D,G] = EstimatedSystem(X)

global a1 l1 m1 I1 d1 a2 l2 m2 I2 d2
global phi1 phi2 phi3 g
global Ox Oz

% retrive the state
th1 = X(1);
th2 = X(2);
dth1 = X(3);
dth2 = X(4);

% System matrix (HOW TO MAKE UNCERTAINTY)
M = [phi1+phi2+2*phi3*cos(th2) phi2+phi3*cos(th2);
    phi2+phi3*cos(th2) phi2];
H = [dth2 dth1+dth2;
    -dth1 0];
D = [d1 0;
    0 d2];
G = g*[m1*l1*cos(th1)+m2*(a1*cos(th1)+l2*cos(th1+th2));
    m2*l2*cos(th1+th2)];

end