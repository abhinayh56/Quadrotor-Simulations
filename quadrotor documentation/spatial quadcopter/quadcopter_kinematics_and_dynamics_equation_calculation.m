format compact; clc; clear;

syms x y z phi th psi dx_dt dy_dt dz_dt dphi_dt dth_dt dpsi_dt real
syms d2x_dt2 d2y_dt2 d2z_dt2 d2phi_dt2 d2th_dt2 d2psi_dt2 real

Rx = [1,        0,         0;
      0, cos(phi), -sin(phi);
      0, sin(phi),  cos(phi)];
 
Ry = [cos(th), 0, sin(th);
            0, 1,       0;
     -sin(th), 0, cos(th)];

 Rz = [cos(psi), -sin(psi), 0;
       sin(psi),  cos(psi), 0;
              0,         0, 1];
R = Rz*Ry*Rx;

T = [1,         0, -sin(th);
     0,  cos(phi),  cos(th)*sin(phi);
     0, -sin(phi),  cos(th)*cos(phi)];

PSI = [psi th phi]';
dPSI_dt = [dphi_dt dth_dt dpsi_dt]';
syms p q r real
% Wb = [p q r]';
Wb = T*dPSI_dt;
Wf = R*Wb;

X = [x y z phi th psi dx_dt dy_dt dz_dt dphi_dt dth_dt dpsi_dt]';
dX_dt = [dx_dt dy_dt dz_dt dphi_dt dth_dt dpsi_dt d2x_dt2 d2y_dt2 d2z_dt2 d2phi_dt2 d2th_dt2 d2psi_dt2]';

syms Ixx Ixy Ixz real
syms Iyx Iyy Iyz real
syms Izx Izy Izz real

I = [Ixx Ixy Ixz;
     Iyx Iyy Iyz;
	 Izx Izy Izz];
 
syms U1 U2 U3 U4 real
syms Thrust Moment real
Thrust = U1;
Moment = [U2 U3 U4]';
syms m g l real
syms Ax Ay Az real
Fd = -[Ax*dx_dt Ay*dy_dt Az*dz_dt]';
Fg = m*[0 0 -g]';
Ft = [0 0 Thrust]';
% r = [x y z]'
d2r_dt2 = (1/m)*(Fg + Fd + R*Ft);

% dT_dt = diff(T,phi)*dphi_dt + diff(T,psi)*dpsi_dt + diff(T,th)*dth_dt;
dT_dt = [0,                 0,                                      -dth_dt*cos(th);
         0, -dphi_dt*sin(phi),   dphi_dt*cos(phi)*cos(th) - dth_dt*sin(phi)*sin(th);
         0, -dphi_dt*cos(phi), - dphi_dt*cos(th)*sin(phi) - dth_dt*cos(phi)*sin(th)];

% d2PSI_dt2 = inv(T)*(inv(I)*(Moment - cross(Wb,I*Wb)) - dT_dt*dPSI_dt)
d2PSI_dt2 = T\(I\(Moment - cross(Wb,I*Wb)) - dT_dt*dPSI_dt);







