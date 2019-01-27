clear all; format long; clf; clc

l=1;
M=1;
g=9.8;

t0=0; tT=20;
time=[t0 tT];
x0=0; vx0=0;
y0=0; vy0=0;
th0=deg2rad(0); omega0=0;
initialValues=[x0 vx0 y0 vy0 th0 omega0];

[t,state]=ode45(@dynamics2Dlin,time,initialValues);
x=state(:,1);
y=state(:,3);
theta=state(:,5);
omega=state(:,6);

% figure(1)
% for i=1:length(t)
%     XX=[x0 x(i)];
%     YY=[y0 y(i)];
%     plot(XX(1),YY(1),'k s',XX,YY,'-',XX(end),YY(end),'r o')
%     axis equal
%     xmin=-1.5; xmax=1.5; ymin=-1.5; ymax=1.5;
%     axis([xmin xmax ymin ymax])
%     drawnow
% end

figure(1)
subplot(2,2,1)
plot(t,x)
title('x/t')
xlabel('t')
ylabel('x')

subplot(2,2,3)
plot(t,y)
title('y/t')
xlabel('t')
ylabel('y')

subplot(2,2,2)
plot(t,theta)
title('th/t')
xlabel('t')
ylabel('th')

subplot(2,2,4)
plot(t,omega)
title('omega/t')
xlabel('t')
ylabel('omega')

figure(2)
plot(x,y,'ko')
axis equal

figure(3)
for i=1:length(state)
    plot(x(i),y(i),'k')
    hold on
%     air frame
    XX=[x(i)-l*cos(theta(i)), x(i)+l*cos(theta(i))];
    YY=[y(i)-l*sin(theta(i)), y(i)+l*sin(theta(i))];
    plot(XX,YY,'b-','lineWidth',1)
    XX0=[x(i)-(l/4.5)*cos(theta(i)), x(i)+(l/4.25)*cos(theta(i))];
    YY0=[y(i)-(l/4.5)*sin(theta(i)), y(i)+(l/4.25)*sin(theta(i))];
    plot(XX0,YY0,'b-','lineWidth',3)
    
%     motor left
    XX1=[XX(1), XX(1)+(l/5)*cos(theta(i)+pi/2)];
    YY1=[YY(1), YY(1)+(l/5)*sin(theta(i)+pi/2)];
    plot(XX1,YY1,'b-','lineWidth',1)
%     motor right
    XX2=[XX(2), XX(2) + (l/5)*cos(theta(i)+pi/2)];
    YY2=[YY(2), YY(2) + (l/5)*sin(theta(i)+pi/2)];
    plot(XX2,YY2,'b-','lineWidth',1)
    
%     prop left
    XX11=[XX1(2)-(l/5)*cos(theta(i)), XX1(2)+(l/5)*cos(theta(i))];
    YY11=[YY1(2)-(l/5)*sin(theta(i)), YY1(2)+(l/5)*sin(theta(i))];
    plot(XX11,YY11,'b-','lineWidth',1)
%     prop right
    XX22=[XX2(2)-(l/5)*cos(theta(i)), XX2(2) + (l/5)*cos(theta(i))];
    YY22=[YY2(2)-(l/5)*sin(theta(i)), YY2(2) + (l/5)*sin(theta(i))];
    plot(XX22,YY22,'b-','lineWidth',1)
    
%     left stand
    XX111=[x(i)-(l/4.25)*cos(theta(i)), x(i)-(l/4.25)*cos(theta(i))-(l/4)*cos(theta(i)+pi/2), x(i)-(l/4.25)*cos(theta(i))-(l/4)*cos(theta(i)+pi/2)+(l/4)*cos(theta(i)+3*pi/2-pi/3)];
    YY111=[y(i)-(l/4.25)*sin(theta(i)), y(i)-(l/4.25)*sin(theta(i))-(l/4)*sin(theta(i)+pi/2), y(i)-(l/4.25)*sin(theta(i))-(l/4)*sin(theta(i)+pi/2)+(l/4)*sin(theta(i)+3*pi/2-pi/3)];
    plot(XX111,YY111,'b-','lineWidth',1)
    
%     right stand
    XX111=[x(i)+(l/4.25)*cos(theta(i)), x(i)+(l/4.25)*cos(theta(i))-(l/4)*cos(theta(i)+pi/2), x(i)+(l/4.25)*cos(theta(i))-(l/4)*cos(theta(i)+pi/2)+(l/4)*cos(theta(i)+3*pi/2+pi/3)];
    YY111=[y(i)+(l/4.25)*sin(theta(i)), y(i)+(l/4.25)*sin(theta(i))-(l/4)*sin(theta(i)+pi/2), y(i)+(l/4.25)*sin(theta(i))-(l/4)*sin(theta(i)+pi/2)+(l/4)*sin(theta(i)+3*pi/2+pi/3)];
    plot(XX111,YY111,'b-','lineWidth',1)
    
    hold off
    axis equal
    xmin=min(x)-5; xmax=max(x)+5; ymin=min(y)-5; ymax=max(y)+5;
    axis([xmin xmax ymin ymax])
    drawnow
end
% syms phi theta psi real
% R1=[cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];
% R2=[cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
% R3=[cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
% R=R1*R2*R3