clear; format long; clf; clc

g = 9.81;
l1 = 0.25;
l2 = 0.5;
m1 = 1.5;
m2 = 0.25;
b = 0.05;
l = l1;
L = l2;

t0=0; tT=15; dt=0.05;
time=t0:dt:tT;
x0=-5; vx0=0;
y0=0; vy0=0;
th0=deg2rad(0); vth0=0;
phi0=deg2rad(-90); vphi0=0;
initialValues=[x0 y0 th0 phi0 vx0 vy0 vth0 vphi0];

[t,state]=ode45(@dynamicQuadrotorSimplePendulumControl,time,initialValues);
x=state(:,1);
y=state(:,2);
theta=state(:,3);
phi=state(:,4);

figure(1)
subplot(2,2,1)
plot(t,x)
title('x/t')
xlabel('t')
ylabel('x')

subplot(2,2,2)
plot(t,y)
title('y/t')
xlabel('t')
ylabel('y')

subplot(2,2,3)
plot(t,rad2deg(theta))
title('th/t')
xlabel('t')
ylabel('th')

subplot(2,2,4)
plot(t,rad2deg(phi))
title('phi/t')
xlabel('t')
ylabel('phi')

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
    
%     pendulum
    XP=[x(i) x(i)+L*cos(phi(i))];
    YP=[y(i) y(i)+L*sin(phi(i))];
    plot(XP,YP,'k-')
    
    hold off
    axis equal
    xmin=min(x)-3; xmax=max(x)+3; ymin=min(y)-3; ymax=max(y)+3;
    axis([xmin xmax ymin ymax])
    grid on
    drawnow
end
% syms phi theta psi real
% R1=[cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];
% R2=[cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
% R3=[cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
% R=R1*R2*R3