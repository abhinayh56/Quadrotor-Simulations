function statedot=dynamics3Dhovercontrollfinal(t,state)
% constants
g=9.8;
l=0.1;
M=1;
I=[1 0 0; 0 1 0; 0 0 1];

% setpoints
x0=1; vx0=0; accx0=0;
y0=-1; vy0=0; accy0=0;
z0=0; vz0=0; accz0=0;
psi0=deg2rad(90); vpsi0=deg2rad(0); accpsi0=deg2rad(0);
phi0=deg2rad(0); vphi0=deg2rad(0); accphi0=deg2rad(0);
theta0=deg2rad(0); vtheta0=deg2rad(0); acctheta0=deg2rad(0);


% rotation matrix ZXY
psi=state(11); %Z
phi=state(7); %X
theta=state(9); %Y

R1=[cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
R2=[1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
R3=[cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
R=R1*R2*R3;

% controll inputs
Kpx=0.3;
Kdx=0.7;
Kpy=0.275;
Kdy=0.65;
Kpz=1;
Kdz=1.2;
Kpphi=4;
Kdphi=2;
Kptheta=4;
Kdtheta=2;
Kppsi=3.5;
Kdpsi=2.4;
x=state(1); vx=state(2);
y=state(3); vy=state(4);
z=state(5); vz=state(6);
vpsi=state(12);
vphi=state(8);
vtheta=state(10);

% control equation
accx0=Kpx*(x0-x)+Kdx*(vx0-vx);
accy0=Kpy*(y0-y)+Kdy*(vy0-vy);
theta0=(1/g)*( accx0*cos(psi0) + accy0*sin(psi0) );
phi0=(1/g)*( accx0*sin(psi0) - accy0*cos(psi0) );
u1=[0; 0; M*(g+accz0) + Kpz*(z0-z) + Kdz*(vz0-vz)];
u2=[Kpphi*(phi0-phi)  + Kdphi*(phi0-vphi);
    Kptheta*(theta0-theta) + Kdtheta*(vtheta0-vtheta);
    Kppsi*(psi0-psi) + Kdpsi*(vpsi0-vpsi)];




% force equation in ground frame
acc = [0; 0; -g] + R*u1;

% moment equation in body frame
% MM=[Kf*l*((w1)^2 - (w3)^2); Kf*l*((w2)^2 - (w4)^2); -Kt*((w1)^2 - (w2)^2 + (w3)^2 - (w4)^2)];
NN=cross([state(6);state(8);state(10)],I*[state(6);state(8);state(10)]);
alpha = inv(I)*(u2-NN);


statedot(1)=state(2);
statedot(2)=acc(1);
statedot(3)=state(4);
statedot(4)=acc(2);
statedot(5)=state(6);
statedot(6)=acc(3);
statedot(7)=state(8);
statedot(8)=alpha(1);
statedot(9)=state(10);
statedot(10)=alpha(2);
statedot(11)=state(12);
statedot(12)=alpha(3);
statedot=statedot';
end