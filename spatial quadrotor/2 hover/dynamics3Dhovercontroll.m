function statedot=dynamics3Dhovercontroll(t,state)
% constants
g=9.8;
l=0.1;
M=1;
I=[1 0 0; 0 1 0; 0 0 1];

% setpoints
x0=0; vx0=0; accx0=0;
y0=0; vy0=0; accy0=0;
z0=0; vz0=0; accz0=0;
psi0=deg2rad(0); vpsi0=deg2rad(0); accpsi0=deg2rad(0);
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

%% 
% % inputs
% p=0.5;
% % vertical
% % hover
% w=sqrt(M*g)*[p sqrt(0.5-p^2) p sqrt(0.5-p^2)]; 
% w1=w(1);
% w2=w(2);
% w3=w(3);
% w4=w(4);
%% 


%% 
% controll inputs
Kpx=0.5;
Kdx=0.5;
Kpy=0.5;
Kdy=0.5;
Kpz=0.1;
Kdz=0.475;
Kpphi=0.1;
Kdphi=0.7;
Kptheta=0.1;
Kdtheta=0.7;
Kppsi=0.2;
Kdpsi=0.55;
x=state(1); vx=state(2);
y=state(3); vy=state(4);
z=state(5); vz=state(6);
vpsi=state(12);
vphi=state(8);
vtheta=state(10);
theta0=(1/g)*( accx0*cos(psi0) + accy0*sin(psi) );
phi0=(1/g)*( accx0*sin(psi0) - accy0*cos(psi) );
u1=[0; 0; M*(g+accz0) + Kpz*(z0-z) + Kdz*(vz0-vz)];
u2=[Kpy*(y0-y) + Kdy*(y0-vy);
    Kpx*(x0-x) + Kdx*(vx0-vx);
    Kppsi*(psi0-psi) + Kdpsi*(vpsi0-vpsi)];
%% 



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