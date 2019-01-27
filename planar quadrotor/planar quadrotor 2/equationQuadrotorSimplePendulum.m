format compact; clear all; clc;
syms F1 F2 u1 u2
syms m1 m2 l1 l2 I1 I2 b g
syms x y th1 th2 xd yd th1d th2d
% g = 9.81;

% -(F1+F2)*sin(th1) = m1*xdd + m2*(xdd - th2dd*l2*sin(th2) - (th2d^2)*l2*cos(th2));
% (F1+F2)*cos(th1) - m1*g - m2*g = m1*ydd + m2*(ydd + th2dd*l2*cos(th2) - (th2d^2)*l2*sin(th2));
% F1*l1 - F2*l1 = I1*th1dd;
% m2*xdd*l2*sin(th2) - (m2*ydd + m2*g)*l2*cos(th2) = I2*th2dd;

% A = [m1+m2         ,  0             , 0 , -m2*l2*sin(th2);
%      0             ,  m1+m2         , 0 ,  m2*l2*cos(th2);
%      m2*l2*sin(th2), -m2*l2*cos(th2), 0 , -I2            ;
%      0             ,  0             , I1, 0               ]
%  
% B = [ m2*l2*(th2d^2)*cos(th2)            ;
%      -(m1+m2)*g + m2*l2*(th2d^2)*sin(th2);
%      m2*l2*g*cos(th2) + b*(th2d-th1d)     ;
%      b*(th2d-th1d)                         ]
% 
% C = [-sin(th1), 0;
%      cos(th1) , 0;
%      0        , 0;
%      0        , 1]
%  
% U = [ F1 + F2   ;
%      (F1 - F2)*l1]

% A = [m1+m2         ,  0             , 0 , -m2*l2*(-1);
%      0             ,  m1+m2         , 0 ,  m2*l2*(th2+pi/2);
%      m2*l2*(-1), -m2*l2*(th2+pi/2), 0 , -I2            ;
%      0             ,  0             , I1, 0               ]
%  
% B = [ m2*l2*(th2d^2)*(th2+pi/2)            ;
%      -(m1+m2)*g + m2*l2*(th2d^2)*(-1);
%      m2*l2*g*(th2+pi/2) + b*(th2d-th1d)     ;
%      b*(th2d-th1d)                         ]
% 
% C = [-th1, 0;
%      1   , 0;
%      0   , 0;
%      0   , 1]
%  
% U = [(m1+m2)*g;
%      0        ]

A = [m1+m2         ,  0             , 0 , -m2*l2*(-1);
     0             ,  m1+m2         , 0 ,  m2*l2*(th2+pi/2);
     m2*l2*(-1), -m2*l2*(th2+pi/2), 0 , -I2            ;
     0             ,  0             , I1, 0               ]
 
B = [ 0                 ;
     -(m1+m2)*g         ;
      m2*l2*g*(th2+pi/2);
      0                 ]

C = [-th1, 0;
     1   , 0;
     0   , 0;
     0   , 1]
 
U = [u1;
     u2]

X = inv(A)*B + inv(A)*C*U