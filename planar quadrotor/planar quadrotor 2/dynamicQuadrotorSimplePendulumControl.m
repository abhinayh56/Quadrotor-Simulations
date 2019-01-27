function Xd=dynamicQuadrotorSimplePendulum(t,X)
% X = [x, y, th1, th2, xd, yd, th1d, th2d];
% Xd = [xd, yd, th1d, th2d, xdd, ydd, th1dd, th2dd];
g = 9.81;
l1 = 0.25;
l2 = 0.5;
m1 = 1.5;
m2 = 0.25;
I1 = m1*l1^2;
I2 = m2*l2^2;
b = 0.05;

x = X(1);
y = X(2);
th1 = X(3);
th2 = X(4);
xd = X(5);
yd = X(6);
th1d = X(7);
th2d = X(8);

% F1 = (m1+m2)*g/2;
% F2 = (m1+m2)*g/2;

% Kpy = 4.5;
% Kdy = 3.5;
% Kpx = 0.75;
% Kdx = 0.95;
% Kpth1 = 10;
% Kdth1 = 8;
% Kpth2 = 5;
% Kdth2 = 3;

Kpy = 4.5;
Kdy = 3.5;

Kpx = 0.4;
Kdx = 1.5;
Kpth1 = 9;
Kdth1 = 4;

Kpth2 = 5;
Kdth2 = 2;

% Kpy = 4.5;
% Kdy = 3.5;
% 
% Kpx = 0;
% Kdx = 0;
% Kpth1 = 0;
% Kdth1 = 0;
% 
% Kpth2 = 0;
% Kdth2 = 0;

% xD = 5;
% yD = 5;
% % th1D = 0;
% th2D = -pi/2;
% xdD = 0;
% ydD = 0;
% th1dD = 0;
% th2dD = 0;
% xddD = 0;
% yddD = 0;
% th1ddD = 0;
% th2ddD = 0;

% xD = 5*cos(t);
% yD = 5*sin(t);
% % th1D = 0;
% th2D = -pi/2;
% xdD = -5*sin(t);
% ydD = 5*cos(t);
% th1dD = 0;
% th2dD = 0;
% xddD = -5*cos(t);
% yddD = -5*sin(t);
% th1ddD = 0;
% th2ddD = 0;

xD = 0;
yD = 0;
% th1D = 0;
th2D = -pi/2;
xdD = 0;
ydD = 0;
th1dD = 0;
th2dD = 0;
xddD = 0;
yddD = 0;
th1ddD = 0;
th2ddD = 0;

th1D = (-1/g)*((xddD + Kpx*(xD-x) + Kdx*(xdD-xd)) + (m2*l2/(m1+m2))*(th2ddD + Kpth2*(th2-th2) + Kdth2*(th2dD-th2d)));
u2 = I1*(th1ddD + Kpth1*(th1D-th1) + Kdth1*(th1dD-th1d));
u1 = (m1+m2)*(g + yddD + Kpy*(yD-y) + Kdy*(ydD-yd))  +  m2*l2*(th2+pi/2)*(th2ddD + Kpth2*(th2D-th2) + Kdth2*(th2dD-th2d));

F1 = (u1 + (u2/l1))/2;
F2 = (u1 - (u2/l1))/2;

A = [m1+m2         ,  0             , 0 , -m2*l2*sin(th2);
     0             ,  m1+m2         , 0 ,  m2*l2*cos(th2);
     m2*l2*sin(th2), -m2*l2*cos(th2), 0 , -I2            ;
     0             ,  0             , I1, 0               ];
 
B = [ m2*l2*(th2d^2)*cos(th2);
     -(m1+m2)*g + m2*l2*(th2d^2)*sin(th2);
     m2*g*l2*cos(th2) + b*(th2d-th1d);
     b*(th2d-th1d)];

C = [-sin(th1), 0;
     cos(th1) , 0;
     0        , 0;
     0        , 1];
 
U = [ F1 + F2   ;
     (F1 - F2)*l1];
 
XX = inv(A)*B + inv(A)*C*U;

Xd(1) = X(5);
Xd(2) = X(6);
Xd(3) = X(7);
Xd(4) = X(8);
Xd(5)=XX(1);
Xd(6)=XX(2);
Xd(7)=XX(3);
Xd(8)=XX(4);
Xd=Xd';
end