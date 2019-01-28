function [ dX_dt ] = planar_Quad_function( t,X )
    
    %% Order of states of the system
    % States (X) of the system are assumed in following order
    % X = [x y th dx_dt dy_dt dth_dt.]
    %% 
    
    %% Robot and Environment parameters
    m = 1; % total mass of quadcopter
    L = 0.25; % distance from CM (center of mass) of quadcopter to the center of propellor
    g = 9.81; % acceleration due to gravity
    Izz = (1/12)*m*L^2; % moment of inertia about CM of quadcopter of quadcopter assuming it as a rigid uniform bar
    %% 
    
    %% Assigning states to variable for ease of writing equations
    x = X(1);
    y = X(2);
    th = X(3);
    Vx = X(4);
    Vy = X(5);
    Vth = X(6);
    %%
    
    %% Desired trajectory of the quadcopter
    x_des = 3;
    y_des = 0;
    Vx_des = 0;
    Vy_des = 0;
    Vth_des = 0;
    Ax_des = 0;
    Ay_des = 0;
    %%

    %% Proporitonality constants for controller (uncomment the situation which ever is required)
    % 1) For critically damped situation
%     Kpx = 0.15;
%     Kdx = 0.55;
%     Kpy = 0.5;
%     Kdy = 2;
%     Kpth = 0.8;
%     Kdth = 0.35;

    % 2) For underdamped agility
    Kpx = 0.7;
    Kdx = 1;
    Kpy = 0.35;
    Kdy = 0.85;
    Kpth = 6;
    Kdth = 3.25;
    %%

    %% Controll inputs to the system
    th_des = (-1/g)*(Ax_des + Kpx*(x_des-x) + Kdx*(Vx_des-Vx)); % deisred angle calculated from one equation of dynamics in X direction of fixed frame
    U1 = m*(g + Ay_des + Kpy*(y_des-y) + Kdy*(Vy_des-Vy)); % thrust input
    U2 = Izz*(Kpth*(th_des-th) + Kdth*(Vth_des-Vth)); % moment input
    
    % limiting inputs withing the saturation limit
    U1 = min(max(0,U1),2*m*g); % equivalent to 0 < U1 < 2mg
    U2 = min(max(-m*g*L,U2),m*g*L); % equivalent to -mgL < U2 < mgL
    %%
    
    %% Set of linear differential equations of the system
    dX_dt(1) = X(4);
    dX_dt(2) = X(5);
    dX_dt(3) = X(6);
    dX_dt(4) = -(U1/m)*sin(th);
    dX_dt(5) = -g + (U1/m)*cos(th);
    dX_dt(6) = U2/Izz;
    dX_dt = dX_dt'; % function should return column matrix
end