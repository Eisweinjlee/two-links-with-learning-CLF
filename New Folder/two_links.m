clear
clc
%% variable of model
syms th1 th2 dth1 dth2 ddth1 ddth2
syms Ox Oz dOx dOz ddOx ddOz
syms a1 m1 l1 I1 d1
syms a2 m2 l2 I2 d2

q=[th1;th2];
dq=[dth1;dth2];
ddq=[ddth1;ddth2];

q_dq=[q;dq];
dq_ddq=[dq;ddq];

%% parameter of model
%%%%%%%%%%%%%%%%%%%%%%%%% Whole system %%%%%%%%%%%%%%%%%%%%%%%%%
Ox=0;
Oz=0;
g=9.8;
% %%%%%%%%%%%%%%%%%%%%%%%%% Link1 %%%%%%%%%%%%%%%%%%%%%%%%%
% a1=0.5;
% l1=1.0;
% m1=10;
% I1=5;
% d1=0;
% %%%%%%%%%%%%%%%%%%%%%%%%% Link2 %%%%%%%%%%%%%%%%%%%%%%%%%
% a2=0.25;
% l2=0.5;
% m2=5;
% I2=0.5;
% d2=0;

%% Homogeneous Transformation Matrix(2D-XZ)
%%%%%%%%%%%%%%%%%%%%%%%%% Link1 to World %%%%%%%%%%%%%%%%%%%%%%%%%
T10=[cos(th1),-sin(th1),0;
    sin(th1),cos(th1),0;
    0,0,1];
%%%%%%%%%%%%%%%%%%%%%%%%% Link2 to Link1 %%%%%%%%%%%%%%%%%%%%%%%%%
T21=[cos(th2),-sin(th2),l1;
    sin(th2),cos(th2),0;
    0,0,1];

%% Calculation of Lagrange
%%%%%%%%%%%%%%%%%%%%%%%%% Link1 %%%%%%%%%%%%%%%%%%%%%%%%%
xzy_1=T10*[a1;0;1];
dxzy_1=(diff(xzy_1,th1)*dth1);
%dxzy_1=(diff(xzy_1,Ox)*dOx)+(diff(xzy_1,Oz)*dOz)+(diff(xzy_1,th1)*dth1);
P_1=m1*g*xzy_1(2);
K_1=1/2*m1*(dxzy_1(1)^2 + dxzy_1(2)^2) + 1/2*I1*dth1^2;
L_1=K_1-P_1;
%%%%%%%%%%%%%%%%%%%%%%%%% Link2 %%%%%%%%%%%%%%%%%%%%%%%%%
xzy_2=T10*T21*[a2;0;1];
%dxzy_2=(diff(xzy_2,Ox)*dOx)+(diff(xzy_2,Oz)*dOz)+(diff(xzy_2,th1)*dth1)+(diff(xzy_2,th2)*dth2);
dxzy_2=(diff(xzy_2,th1)*dth1)+(diff(xzy_2,th2)*dth2);
P_2=m2*g*xzy_2(2);
K_2=1/2*m2*(dxzy_2(1)^2 + dxzy_2(2)^2) + 1/2*I2*(dth1+dth2)^2;
L_2=K_2-P_2;

%% Equation of Lagrange
LL=L_1+L_2;

for i=1:1:2
   dL_dq(i,:)=diff(LL,dq(i));
   dL_q(i,:)=diff(LL,q(i));
   dLdq_dt(i,:)=jacobian(dL_dq(i,:),q_dq)*dq_ddq;
   %dD_all_dq(i,:)=diff(D_all,dq(i));
   %Tau(i,:)=dLdq_dt(i,:)-dL_q(i,:)+dD_all_dq(i,:);
   Tau(i,:)=dLdq_dt(i,:)-dL_q(i,:);
end

M=jacobian(Tau,ddq);
C=Tau-M*ddq;
% size(M)
% size(C)
simplify(M)
simplify(C)