format long; format compact; clear; clf; clc % clearing previous activities

%% Solving differential equations for the system using ode45 function
tspan = 0:0.1:10; % time span for wich differential equations are solved
% initial condition of quadrotor
x0 = 0;
y0 = 0;
z0 = 0;
phi0 = 0;
th0 = 0;
psi0 = 0;
Vx0 = 0;
Vy0 = 0;
Vz0 = 0;
Vphi0 = 0;
Vth0 = 0;
Vpsi0 = 0;

x0 = [x0, y0, z0, phi0, th0, psi0, Vx0, Vy0, Vz0, Vphi0, Vth0, Vpsi0]; % initial condition state vector
% fname = 'spatial_Quad_function';
% [t,states] = ode45(fname,tspan,x0);
[t,states] = ode45(@spatial_Quad_function,tspan,x0); % solving nonlinear differential equations with ode45 function
%%

%% Assigning states to state variables for simplicity in handling
x = states(:,1);
y = states(:,2);
z = states(:,3);
phi = states(:,4);
th = states(:,5);
psi = states(:,6);
Vx = states(:,7);
Vy = states(:,8);
Vz = states(:,9);
Vphi = states(:,10);
Vth = states(:,11);
Vpsi = states(:,12);
%%

%% Plotting variation of states in graphical form
figure(1) % figure 1 plots translational states
% position along x axis in Inertial frame
subplot(2,3,1);
plot(t,x,'b')
title('x-t')
xlabel('t (s)')
ylabel('x (m)')

% position along y axis in Inertial frame
subplot(2,3,2);
plot(t,y,'b')
title('y-t')
xlabel('t (s)')
ylabel('y (m)')

% position along z axis in Inertial frame
subplot(2,3,3);
plot(t,z,'b')
title('z-t')
xlabel('t (s)')
ylabel('z (m)')

% linear velocity along x axis in Inertial frame
subplot(2,3,4);
plot(t,Vx,'b')
title('Vx-t')
xlabel('t (s)')
ylabel('Vx (ms^{-1})')

% linear velocity along y axis in Inertial frame
subplot(2,3,5);
plot(t,Vy,'b')
title('Vy-t')
xlabel('t (s)')
ylabel('Vy (ms{-1})')

% linear velocity along z axis in Inertial frame
subplot(2,3,6);
plot(t,Vz,'b')
title('Vz-t')
xlabel('t (s)')
ylabel('Vz (ms{-1})')

figure(2) % figure 2 plots rotational states
% orientation along x axis in Intermediate frame
subplot(2,3,1);
plot(t,phi*180/pi,'b')
title('phi-t')
xlabel('t (s)')
ylabel('phi (degree)')

% orientation along y axis in Intermediate frame
subplot(2,3,2);
plot(t,th*180/pi,'b')
title('th-t')
xlabel('t (s)')
ylabel('th (degree)')

% orientation along z axis in Inertial frame
subplot(2,3,3);
plot(t,psi*180/pi,'b')
title('psi-t')
xlabel('t (s)')
ylabel('psi (degree)')

% angular velocity about x axis in Intermediate frame
subplot(2,3,4);
plot(t,Vphi*180/pi,'b')
title('Vphi-t')
xlabel('t (s)')
ylabel('Vphi (degree/s)')

% angular velocity about y axis in Intermediate frame
subplot(2,3,5);
plot(t,Vth*180/pi,'b')
title('Vth-t')
xlabel('t (s)')
ylabel('Vth (degree/s)')

% angular velocity about z axis in Inertial frame
subplot(2,3,6);
plot(t,Vpsi*180/pi,'b')
title('Vpsi-t')
xlabel('t (s)')
ylabel('Vpsi (degree/s)')
%%

%% Model of quadcopter in 3D
%--------------------------------------------------------------------------
% The quadcopter is in + configuration
%             O                      
%             |                      o                  o                      
%             |                      |_ _ _ _ __ _ _ _ _|       
%     O- - - - - - - -O                     |    |     
%             |                            /      \
%             |                       
%             O
%          top view                       side view
%--------------------------------------------------------------------------
%                        notations
%                        B (+Y-axis of body frame)
%                       O
%                       |
%                       |         A
%               O- - - - - - - -O  (+X-axis of body frame)
%              C        |
%                       |
%                       O
%                        D
%--------------------------------------------------------------------------
%%

%% Visualization of the qudcopter in simulation
figure(3)
for i=1:length(t)
    plot3(x(1:i),y(1:i),z(1:i),':m','lineWidth',2) % trail left behind the quadcopter during flight
    hold on
    
    % Display Robotics lab
    H_l = 11*60/100; % in meter
    H_b = 9*60/100;
    H_h = 3.6576; % 12 ft
    H_floor_X = [H_l/2, -H_l/2, -H_l/2, H_l/2, H_l/2];
    H_floor_Y = [H_b/2, H_b/2, -H_b/2, -H_b/2, H_b/2];
    H_floor_Z = [0, 0, 0, 0, 0];
    plot3(H_floor_X,H_floor_Y,H_floor_Z, 'k-')
    fill3(H_floor_X,H_floor_Y,H_floor_Z,[178/255,178/255,178/255])
    alpha(1) %transparency
    % inertial frame
    frame_l = 0.5;
    plot3([0,frame_l],[0,0],[0,0],'r-','LineWidth',1)
    plot3([0,0],[0,frame_l],[0,0],'g-','LineWidth',1)
    plot3([0,0],[0,0],[0,frame_l],'b-','LineWidth',1)
    plot3(0,0,0,'k.')
    % roof
    H_roof_X = [H_l/2, -H_l/2, -H_l/2, H_l/2, H_l/2];
    H_roof_Y = [H_b/2, H_b/2, -H_b/2, -H_b/2, H_b/2];
    H_roof_Z = [H_h, H_h, H_h, H_h, H_h];
    plot3(H_roof_X,H_roof_Y,H_roof_Z, 'k-')
    % linter
    plot3(H_roof_X,H_roof_Y,H_roof_Z*2/3,'k-','LineWidth',1)
    % beam 1
    beam_1_X = [H_l/2, H_l/2];
    beam_1_Y = [H_b/2, H_b/2];
    beam_1_Z = [0, H_h];
    plot3(beam_1_X,beam_1_Y,beam_1_Z,'k-','LineWidth',1)
    % beam 2
    beam_2_X = [-H_l/2, -H_l/2];
    beam_2_Y = [H_b/2, H_b/2];
    beam_2_Z = [0, H_h];
    plot3(beam_2_X,beam_2_Y,beam_2_Z,'k-','LineWidth',1)
    % beam 3
    beam_3_X = [-H_l/2, -H_l/2];
    beam_3_Y = [-H_b/2, -H_b/2];
    beam_3_Z = [0, H_h];
    plot3(beam_3_X,beam_3_Y,beam_3_Z,'k-','LineWidth',1)
    % beam 4
    beam_4_X = [H_l/2, H_l/2];
    beam_4_Y = [-H_b/2, -H_b/2];
    beam_4_Z = [0, H_h];
    plot3(beam_4_X,beam_4_Y,beam_4_Z,'k-','LineWidth',1)
    
    % Vicon and position vector
    H_h = H_h*2/3;
    plot3([0,x(i)],[0,y(i)],[0,z(i)],'k-') % r vector
    plot3([x(i),x(i)],[y(i),y(i)],[0,0],'k.') % shadow on ground
    % vicon
    plot3([x(i),H_l/2],[y(i),H_b/2],[z(i),H_h],'k:')
    plot3([x(i),-H_l/2],[y(i),H_b/2],[z(i),H_h],'k:')
    plot3([x(i),-H_l/2],[y(i),-H_b/2],[z(i),H_h],'k:')
    plot3([x(i),H_l/2],[y(i),-H_b/2],[z(i),H_h],'k:')
    plot3([x(i),H_l/2],[y(i),0],[z(i),H_h],'k:')
    plot3([x(i),0],[y(i),H_b/2],[z(i),H_h],'k:')
    plot3([x(i),-H_l/2],[y(i),0],[z(i),H_h],'k:')
    plot3([x(i),0],[y(i),-H_b/2],[z(i),H_h],'k:')
    H_h = H_h*3/2;
    
    % Model of quadcopter
    % frame
    L = 0.25;
    Ab = [L; 0; 0];
    Bb = [0; L; 0];
    Cb = [-L; 0; 0];
    Db = [0; -L; 0];
    
    Rx = [1,        0,         0;
          0, cos(phi(i)), -sin(phi(i));
          0, sin(phi(i)),  cos(phi(i))];
 
    Ry = [cos(th(i)), 0, sin(th(i));
             0, 1,       0;
         -sin(th(i)), 0, cos(th(i))];

    Rz = [cos(psi(i)), -sin(psi(i)), 0;
          sin(psi(i)),  cos(psi(i)), 0;
                 0,         0, 1];
    R = Rz*Ry*Rx;
    translation = [x(i); y(i); z(i)];
    Af = translation + R*Ab;
    Bf = translation + R*Bb;
    Cf = translation + R*Cb;
    Df = translation + R*Db;
    
    XX1 = [Af(1) Cf(1)];
    YY1 = [Af(2) Cf(2)];
    ZZ1 = [Af(3) Cf(3)];
    XX2 = [Bf(1) Df(1)];
    YY2 = [Bf(2) Df(2)];
    ZZ2 = [Bf(3) Df(3)];
    red = translation + R*Ab/2;
    XXr = [Af(1) red(1)];
    YYr = [Af(2) red(2)];
    ZZr = [Af(3) red(3)];
    plot3(XX1,YY1,ZZ1,'b-','lineWidth',1)
    plot3(XX2,YY2,ZZ2,'b-','lineWidth',1)
    plot3(XXr,YYr,ZZr,'r-','lineWidth',1) % red color for indicating front part of quadcopter

    % motors
    Amb = [0; 0; L/4];
    Bmb = [0; 0; L/4];
    Cmb = [0; 0; L/4];
    Dmb = [0; 0; L/4];
    Amf = translation + R*(Ab+Amb);
    Bmf = translation + R*(Bb+Bmb);
    Cmf = translation + R*(Cb+Cmb);
    Dmf = translation + R*(Db+Dmb);
    % motor A
    XXmA=[Af(1),Amf(1)];
    YYmA=[Af(2),Amf(2)];
    ZZmA=[Af(3),Amf(3)];
    plot3(XXmA,YYmA,ZZmA,'r-','lineWidth',1)
    % motor B
    XXmB=[Bf(1),Bmf(1)];
    YYmB=[Bf(2),Bmf(2)];
    ZZmB=[Bf(3),Bmf(3)];
    plot3(XXmB,YYmB,ZZmB,'b-','lineWidth',1)
    % motor C
    XXmC=[Cf(1),Cmf(1)];
    YYmC=[Cf(2),Cmf(2)];
    ZZmC=[Cf(3),Cmf(3)];
    plot3(XXmC,YYmC,ZZmC,'b-','lineWidth',1)
    % motor D
    XXmD=[Df(1),Dmf(1)];
    YYmD=[Df(2),Dmf(2)];
    ZZmD=[Df(3),Dmf(3)];
    plot3(XXmD,YYmD,ZZmD,'b-','lineWidth',1)
    
    % stands
    Asb1 = [L/4; 0; 0];
    Bsb1 = [0; L/4; 0];
    Csb1 = [-L/4; 0; 0];
    Dsb1 = [0; -L/4; 0];
    Asf1 = translation + R*Asb1;
    Bsf1 = translation + R*Bsb1;
    Csf1 = translation + R*Csb1;
    Dsf1 = translation + R*Dsb1;
    Asb2 = [L/4; 0; -L/4];
    Bsb2 = [0; L/4; -L/4];
    Csb2 = [-L/4; 0; -L/4];
    Dsb2 = [0; -L/4; -L/4];
    Asf2 = translation + R*Asb2;
    Bsf2 = translation + R*Bsb2;
    Csf2 = translation + R*Csb2;
    Dsf2 = translation + R*Dsb2;
    Asb3 = [L/4; 0; -L/4] + [L/4*cos(-pi/6); 0; L/4*sin(-pi/6)];
    Bsb3 = [0; L/4; -L/4] + [0; L/4*cos(-pi/6); L/4*sin(-pi/6)];
    Csb3 = [-L/4; 0; -L/4] + [L/4*cos(pi+pi/6); 0; L/4*sin(pi+pi/6)];
    Dsb3 = [0; -L/4; -L/4] + [0; L/4*cos(pi+pi/6); L/4*sin(pi+pi/6)];
    Asf3 = translation + R*Asb3;
    Bsf3 = translation + R*Bsb3;
    Csf3 = translation + R*Csb3;
    Dsf3 = translation + R*Dsb3;
    % stand A
    XXsA=[Asf1(1);Asf2(1);Asf3(1)];
    YYsA=[Asf1(2);Asf2(2);Asf3(2)];
    ZZsA=[Asf1(3);Asf2(3);Asf3(3)];
    plot3(XXsA,YYsA,ZZsA,'b-','lineWidth',1)
    % stand B
    XXsB=[Bsf1(1);Bsf2(1);Bsf3(1)];
    YYsB=[Bsf1(2);Bsf2(2);Bsf3(2)];
    ZZsB=[Bsf1(3);Bsf2(3);Bsf3(3)];
    plot3(XXsB,YYsB,ZZsB,'b-','lineWidth',1)
    % stand C
    XXsC=[Csf1(1);Csf2(1);Csf3(1)];
    YYsC=[Csf1(2);Csf2(2);Csf3(2)];
    ZZsC=[Csf1(3);Csf2(3);Csf3(3)];
    plot3(XXsC,YYsC,ZZsC,'b-','lineWidth',1)
    % stand D
    XXsD=[Dsf1(1);Dsf2(1);Dsf3(1)];
    YYsD=[Dsf1(2);Dsf2(2);Dsf3(2)];
    ZZsD=[Dsf1(3);Dsf2(3);Dsf3(3)];
    plot3(XXsD,YYsD,ZZsD,'b-','lineWidth',1)
    
    hold off
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
    completed = i*100/length(t);
    title(strcat('Visualiztion [Completed: ', int2str(round(completed)), '%]'))
    grid on
    grid minor
    j = 1;
    xmin=-H_l/2-j; xmax=H_l/2+j; ymin=-H_b/2-j; ymax=H_b/2+j; zmin=-j; zmax=H_h+j;
    axis([xmin xmax ymin ymax zmin zmax])
    drawnow
end