function statedot=dynamics3D(t,state)
% constnts
g=9.8;
l=0.1;
M=1;
Kf=1;
Kt=1;
I=[1 0 0; 0 1 0; 0 0 1];

% rotation matrix ZXY
psi=state(11); %Z
phi=state(7); %X
theta=state(9); %Y

R1=[cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
R2=[1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
R3=[cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
R=R1*R2*R3;

% inputs
p=0.5001;
% vertical
% w=0.001+sqrt(M*g)*[p sqrt(0.5-p^2) p sqrt(0.5-p^2)];
% hover
w=sqrt(M*g)*[p sqrt(0.5-p^2) p sqrt(0.5-p^2)]; 
w1=w(1);
w2=w(2);
w3=w(3);
w4=w(4);

% force equation in ground frame
acc = [0; 0; -g] + R*[0; 0; Kf*((w1)^2 + (w2)^2 + (w3)^2 + (w4)^2)];

% moment equation in body frame
MM=[Kf*l*((w1)^2 - (w3)^2); Kf*l*((w2)^2 - (w4)^2); -Kt*((w1)^2 - (w2)^2 + (w3)^2 - (w4)^2)];
NN=cross([state(6);state(8);state(10)],I*[state(6);state(8);state(10)]);
alpha = inv(I)*(MM-NN);


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