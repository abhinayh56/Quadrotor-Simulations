clear all; format long; clf; close all; clc

%% 
% time interval
t0=0; tT=78;
time=t0:0.2:tT;
%% 

%% 
% initial values
n=1;
x0=0; vx0=0;
y0=0; vy0=0;
z0=0; vz0=0;
alpha0=deg2rad(0); valpha0=deg2rad(0);
beta0=deg2rad(0); vbeta0=deg2rad(0);
gama0=deg2rad(0); vgama0=deg2rad(0);
initialValues=[x0 vx0 y0 vy0 z0 vz0 alpha0 valpha0 beta0 vbeta0 gama0 vgama0];
%% 

%% 
% solving differential equations
[t,state]=ode45(@dynamics3Dtrajectory,time,initialValues);
%% 

%% 
% saving results
x=state(:,1);
vx=state(:,2);
y=state(:,3);
vy=state(:,4);
z=state(:,5);
vz=state(:,6);
alpha=state(:,7);
valpha=state(:,8);
beta=state(:,9);
vbeta=state(:,10);
gama=state(:,11);
vgama=state(:,12);
%% 

% displaying results
figure(1)
subplot(2,3,1)
plot(t,x)
xlabel('t')
ylabel('x')
title('x-t')
% grid on
axis on

subplot(2,3,2)
plot(t,y)
xlabel('t')
ylabel('y')
title('y-t')

subplot(2,3,3)
plot(t,z)
xlabel('t')
ylabel('z')
title('z-t')

subplot(2,3,4)
plot(t,vx)
xlabel('t')
ylabel('vx')
title('vx-t')

subplot(2,3,5)
plot(t,vy)
xlabel('t')
ylabel('vy')
title('vy-t')

subplot(2,3,6)
plot(t,vz)
xlabel('t')
ylabel('vz')
title('vz-t')

figure(2)
subplot(2,3,1)
plot(t,rad2deg(alpha))
xlabel('t')
ylabel('alpha')
title('alpha-t')

subplot(2,3,2)
plot(t,rad2deg(beta))
xlabel('t')
ylabel('beta')
title('beta-t')

subplot(2,3,3)
plot(t,rad2deg(gama))
xlabel('t')
ylabel('gama')
title('gama-t')

subplot(2,3,4)
plot(t,rad2deg(valpha))
xlabel('t')
ylabel('valpha')
title('valpha-t')

subplot(2,3,5)
plot(t,rad2deg(vbeta))
xlabel('t')
ylabel('vbeta')
title('vbeta-t')

subplot(2,3,6)
plot(t,rad2deg(vgama))
xlabel('t')
ylabel('vgama')
title('vgama-t')

figure(3)
plot3(x,y,z,'b-')
title('trajectory')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal

figure(4)
for i=1:length(t)
    plot3(x(i),y(i),z(i),'k.')
    hold on
    plot3(x,y,z,'k-')
    xlabel('x')
    ylabel('y')
    zlabel('z')
    axis equal
    %% 
    
    %% 
    psi=gama(i); %Z
    phi=alpha(i); %X
    theta=beta(i); %Y
    R1=[cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R2=[1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    R3=[cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    R=R1*R2*R3;
    %% 
    
    %% 
% quadrotor model
    l=0.75;
%     before transformation
    A=[l; 0; 0];
    B=[0; l; 0];
    C=[-l; 0; 0];
    D=[0; -l; 0];
%     angle=linspace(0,2*pi,100);
%     Ac=A+(l/4)*[cos(angle); sin(angle); 0];
%     Bc=B+(l/4)*[cos(angle); sin(angle); 0];
%     Cc=C+(l/4)*[cos(angle); sin(angle); 0];
%     Dc=D+(l/4)*[cos(angle); sin(angle); 0];
    d=[x(i); y(i); z(i)];
    
%     after transformation
    An=R*A + d;
    Bn=R*B + d;
    Cn=R*C + d;
    Dn=R*D + d;
%     Acn=R*Ac + d;
%     Bcn=R*Bc + d;
%     Ccn=R*Cc + d;
%     Dcn=R*Dc + d;
    %% 
    
    %% 
    Anx=An(1);
    Any=An(2);
    Anz=An(3);
    Bnx=Bn(1);
    Bny=Bn(2);
    Bnz=Bn(3);
    Cnx=Cn(1);
    Cny=Cn(2);
    Cnz=Cn(3);
    Dnx=Dn(1);
    Dny=Dn(2);
    Dnz=Dn(3);
%     Acnx=Acn(1);
%     Acny=Acn(2);
%     Acnz=Acn(3);
%     Bcnx=Bcn(1);
%     Bcny=Bcn(2);
%     Bcnz=Bcn(3);
%     Ccnx=Ccn(1);
%     Ccny=Ccn(2);
%     Ccnz=Ccn(3);
%     Dcnx=Dcn(1);
%     Dcny=Dcn(2);
%     Dcnz=Dcn(3);
    %% 
    
    
    %% 
    XX1=[Anx Cnx];
    YY1=[Any Cny];
    ZZ1=[Anz Cnz];
    XX2=[Bnx Dnx];
    YY2=[Bny Dny];
    ZZ2=[Bnz Bnz];
    plot3(XX1,YY1,ZZ1,'b-.')
    plot3(XX2,YY2,ZZ2,'b-.')
    plot3(Anx,Any,Anz,'k.')
%     plot3(Acnx,Acny,Acnz)
%     plot3(Bcnx,Bcny,Bcnz)
%     plot3(Ccnx,Ccny,Ccnz)
%     plot3(Dcnx,Dcny,Dcnz)
    text(x(i),y(i),[num2str(x(i)),num2str(y(i))])
    %% 
    
    %% 
    hold off
%     axis equal
%     axis auto
    xmin=min(x)-0.5; xmax=max(x)+0.5; ymin=min(y)-0.5; ymax=max(y)+0.5; zmin=min(z)-0.5; zmax=max(z)+0.5;
%     axis([xmin xmax ymin ymax zmin zmax])
    drawnow
end
