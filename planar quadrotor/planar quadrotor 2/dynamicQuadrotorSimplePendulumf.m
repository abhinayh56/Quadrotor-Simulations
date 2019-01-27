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
b = 0.005;

F1 = (m1+m2)*g/2;
F2 = (m1+m2)*g/2;

x = X(1);
y = X(2);
th1 = X(3);
th2 = X(4);
xd = X(5);
yd = X(6);
th1d = X(7);
th2d = X(8);

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