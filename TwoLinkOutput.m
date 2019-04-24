function [joint2, EOF, dEOF] = TwoLinkOutput(X)

% function TwoLinkOutput is to calculate the position of joint 2
% and the position of end of effect (EOF)
%
% Input:    X: the state X, 4d real number vector
%
% Return:   joint2: the position of joint 2, 2d real number vector
%           EOF: the position of EOF , 2d real number vector
%           dEOF: the velocity of EOF , 2d real number vector

th1 = X(1);
th2 = X(2);
dth1 = X(3);
dth2 = X(4);

joint2 = [cos(th1);sin(th1)];
EOF = joint2 + 0.5*[cos(th1+th2);sin(th1+th2)];

dEOF = [ - dth1*(sin(th1 + th2)/2 + sin(th1)) - (dth2*sin(th1 + th2))/2;
    dth1*(cos(th1 + th2)/2 + cos(th1)) + (dth2*cos(th1 + th2))/2];

end