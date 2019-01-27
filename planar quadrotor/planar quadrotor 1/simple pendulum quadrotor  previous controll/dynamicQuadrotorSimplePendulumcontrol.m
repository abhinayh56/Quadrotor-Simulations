function statedot=dynamicQuadrotorSimplePendulum(t,state)
g=9.81;
L=1.5;
l=0.5; % frame
m=2;
M=1;

F1=(m+M)*g/2;
F2=(m+M)*g/2;

% kpx=0;
% kdx=0;
% kpy=0;
% kdy=0;
% kpth=0;
% kdth=0;

kpx=0.85;
kdx=1.75;
kpy=3;
kdy=2;
kpth=6;
kdth=4.25;

% kpx=2;
% kdx=1;
% kpy=20;
% kdy=8.55;
% kpth=4;
% kdth=4;


setx=5;
setxv=0;
setxacc=0;
sety=5;
setyv=0;
setyacc=0;
setth=0;
setthv=0;

setth=(-1/g)*(setxacc + kpx*(setx-state(1)) + kdx*(setxv-state(5)));
u1=M*g + m*(g + setyacc + kpy*(sety-state(2)) + kdy*(setyv-state(6)));
u2=kpth*(setth-state(3)) + kdth*(setthv-state(7));
F1=(u1-u2)/2;
F2=(u1+u2)/2;


% A=(F1+F2)/m;
% B=(M/m)*(g*sin(state(4)) + L*(state(8))^2);
% 
% statedot(1)=state(5);
% statedot(2)=state(6);
% statedot(3)=state(7);
% statedot(4)=state(8);
% statedot(5)=-A*sin(state(3)) + B*cos(state(4));
% statedot(6)=A*cos(state(3)) - g - B*sin(state(4));
% statedot(7)=(F2-F1)/(m*l);
% % statedot(8)=-(-A*sin(state(3)) + B*cos(state(4)))*sin(state(4))/l + (g+A*cos(state(3)) - g - B*sin(state(4)))*cos(state(4))/l;
% statedot(8)=-(g + (A*cos(state(3)) - g - B*sin(state(4))))*cos(state(4)) - (-A*sin(state(3)) + B*cos(state(4)))*sin(state(4));
% statedot=statedot';


% statedot(1)=state(5);
% statedot(2)=state(6);
% statedot(3)=state(7);
% statedot(4)=state(8);
% 
% a1=m + M*(cos(state(4)))^2;
% b1=-M*sin(state(4))*cos(state(4));
% c1=-(F1+F2)*sin(state(3)) + M*cos(state(4))*(L*(state(8))^2 + g*sin(state(4)));
% 
% a2=-M*sin(state(4))*cos(state(4));
% b2=m + M*(sin(state(4)))^2;
% c2=(F1+F2)*cos(state(3)) - m*g - M*sin(state(4))*(L*(state(8))^2 + g*sin(state(4)));
% 
% accX=(b2*c1)/(a1*b2 - a2*b1) - (b1*c2)/(a1*b2 - a2*b1);
% accY=(a1*c2)/(a1*b2 - a2*b1) - (a2*c1)/(a1*b2 - a2*b1);
% accPhi=-(accX*sin(state(4)) + cos(state(4))*(g+accY))/L;
% 
% statedot(5)=accX;
% statedot(6)=accY;
% statedot(7)=(F2-F1)/(m*l);
% statedot(8)=accPhi;
% statedot=statedot';

statedot(1)=state(5);
statedot(2)=state(6);
statedot(3)=state(7);
statedot(4)=state(8);

theta=state(3);
phi=state(4);
phiV=state(8);

A=[m,             0,               0,      0,       -cos(phi);
   0,             m,               0,      0,       -sin(phi);
   0,             0,               m*l^2,  0,       0        ;
   M*cos(phi),    M*sin(phi),      0,      0,       1        ;
   M*L*sin(phi),  -M*L*cos(phi),   0,      -M*L^2,  0        ];
B=[-(F1+F2)*sin(theta);
   (F1+F2)*cos(theta)-m*g;
   (F2-F1)*l;
    M*(phiV^2)*L - M*g*sin(phi);
    M*g*L*cos(phi)];

X=inv(A)*B;

statedot(5)=X(1);
statedot(6)=X(2);
statedot(7)=X(3);
statedot(8)=X(4);
statedot=statedot';

end