syms th1 th2 dth1 dth2
y = [cos(th1) + 0.5 * cos(th1+th2);
    sin(th1)+ 0.5 * sin(th1+th2)];
J = jacobian(y, [th1; th2]);
simplify(J)%*[dth1;dth2]

dy = [ - dth1*(sin(th1 + th2)/2 + sin(th1)) - (dth2*sin(th1 + th2))/2;
    dth1*(cos(th1 + th2)/2 + cos(th1)) + (dth2*cos(th1 + th2))/2];
JJ = jacobian(dy, [th1; th2]);
simplify(JJ)