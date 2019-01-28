format compact; clear; clf; clc % clearing previous activities

%% Solving differential equations for the system using ode45 function
tspan = 0:0.1:15; % time span for wich differential equations are solved
x0 = 0;
y0 = 0;
th0 = 0;
vx0 = 0;
vy0 = 0;
omega0 = 0;
x0 = [x0, y0, th0, vx0, vy0, omega0]; % initial condition
% fname = 'planar_Quad_function';
% [t,states] = ode45(fname,tspan,x0);
[t,states] = ode45(@planar_Quad_function,tspan,x0);
%%

%% Assigning states to state variables for simplicity in computing
x = states(:,1);
y = states(:,2);
th = states(:,3);
Vx = states(:,4);
Vy = states(:,5);
omega = states(:,6);
%%

%% Plotting variation of states in graphical form
figure(1)
% position along x axis in Inertial frame
subplot(2,3,1);
plot(t,x,'r')
title('x-t')
xlabel('t (s)')
ylabel('x (m)')

% position along y axis in Inertial frame
subplot(2,3,2);
plot(t,y,'g')
title('y-t')
xlabel('t (s)')
ylabel('y (m)')

% orientation along z axis in Inertial frame
subplot(2,3,3);
plot(t,th*180/pi,'b')
title('th-t')
xlabel('t (s)')
ylabel('th (degree)')

% linear velocity along x axis in Inertial frame
subplot(2,3,4);
plot(t,Vx,'r')
title('Vx-t')
xlabel('t (s)')
ylabel('Vx (ms^{-1})')

% linear velocity along y axis in Inertial frame
subplot(2,3,5);
plot(t,Vy,'g')
title('Vy-t')
xlabel('t (s)')
ylabel('Vy (ms{-1})')

% angular velocity about z axis in Inertial frame
subplot(2,3,6);
plot(t,omega*180/pi,'b')
title('omega-t')
xlabel('t (s)')
ylabel('omega (degree/s)')
%%

%% Model of quadcopter in 2D
% _____                    _____
%   |                        |
%   |                        |
%   ---------[[[[]]]]---------
%            |      |
%           /        \
%          '          '
%%

%% Visualization of the qudcopter in simulation
figure(2)
for i=1:length(t)
    plot(x(1:i),y(1:i),':k') % trail left behind the quadcopter during flight
    hold on
    
    % Model of quadcopter
    
    % air frame
    L = 0.25;
    Ab = [L; 0];
    Bb = [-L; 0];
    R = [cos(th(i)) -sin(th(i)); sin(th(i)) cos(th(i))];
    translation = [x(i); y(i)];
    Af = translation + R*Ab;
    Bf = translation + R*Bb;
    XX = [Af(1) Bf(1)];
    YY = [Af(2) Bf(2)];
    plot(XX,YY,'b-','lineWidth',1)
    XX0=[x(i)-(L/4.5)*cos(th(i)), x(i)+(L/4.25)*cos(th(i))];
    YY0=[y(i)-(L/4.5)*sin(th(i)), y(i)+(L/4.25)*sin(th(i))];
    plot(XX0,YY0,'b-','lineWidth',3)

    % motor left
    XX1=[XX(1), XX(1)+(L/5)*cos(th(i)+pi/2)];
    YY1=[YY(1), YY(1)+(L/5)*sin(th(i)+pi/2)];
    plot(XX1,YY1,'b-','lineWidth',1)
    
    % motor right
    XX2=[XX(2), XX(2) + (L/5)*cos(th(i)+pi/2)];
    YY2=[YY(2), YY(2) + (L/5)*sin(th(i)+pi/2)];
    plot(XX2,YY2,'b-','lineWidth',1)
    
    % prop left
    XX11=[XX1(2)-(L/5)*cos(th(i)), XX1(2)+(L/5)*cos(th(i))];
    YY11=[YY1(2)-(L/5)*sin(th(i)), YY1(2)+(L/5)*sin(th(i))];
    plot(XX11,YY11,'b-','lineWidth',1)
    % prop right
    XX22=[XX2(2)-(L/5)*cos(th(i)), XX2(2) + (L/5)*cos(th(i))];
    YY22=[YY2(2)-(L/5)*sin(th(i)), YY2(2) + (L/5)*sin(th(i))];
    plot(XX22,YY22,'b-','lineWidth',1)
    
    % left stand
    XX111=[x(i)-(L/4.25)*cos(th(i)), x(i)-(L/4.25)*cos(th(i))-(L/4)*cos(th(i)+pi/2), x(i)-(L/4.25)*cos(th(i))-(L/4)*cos(th(i)+pi/2)+(L/4)*cos(th(i)+3*pi/2-pi/3)];
    YY111=[y(i)-(L/4.25)*sin(th(i)), y(i)-(L/4.25)*sin(th(i))-(L/4)*sin(th(i)+pi/2), y(i)-(L/4.25)*sin(th(i))-(L/4)*sin(th(i)+pi/2)+(L/4)*sin(th(i)+3*pi/2-pi/3)];
    plot(XX111,YY111,'b-','lineWidth',1)
    
    % right stand
    XX111=[x(i)+(L/4.25)*cos(th(i)), x(i)+(L/4.25)*cos(th(i))-(L/4)*cos(th(i)+pi/2), x(i)+(L/4.25)*cos(th(i))-(L/4)*cos(th(i)+pi/2)+(L/4)*cos(th(i)+3*pi/2+pi/3)];
    YY111=[y(i)+(L/4.25)*sin(th(i)), y(i)+(L/4.25)*sin(th(i))-(L/4)*sin(th(i)+pi/2), y(i)+(L/4.25)*sin(th(i))-(L/4)*sin(th(i)+pi/2)+(L/4)*sin(th(i)+3*pi/2+pi/3)];
    plot(XX111,YY111,'b-','lineWidth',1)
    
    hold off
    axis equal
    xlabel('x')
    ylabel('y')
    completed = i*100/length(t);
    title(strcat('Visualiztion [Completed: ', int2str(round(completed)), '%]'))
    grid on
    grid minor
    j = 2;
    xmin=min(x)-j; xmax=max(x)+j; ymin=min(y)-j; ymax=max(y)+j;
    axis([xmin xmax ymin ymax])
    drawnow
end