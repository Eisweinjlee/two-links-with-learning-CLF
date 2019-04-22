syms th1 th2 dth1 dth2
y = [cos(th1) + 0.5 * cos(th1+th2);
    sin(th1)+ 0.5 * sin(th1+th2)];
J = jacobian(y, [th1; th2]);
simplify(J)*[dth1;dth2]