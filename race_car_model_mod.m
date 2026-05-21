function model = race_car_model_mod()
    import casadi.*

    %% ==============
    %  Model Name
    % ==============
    model = struct();
    model.name = 'racecar_lap_time_mpc';

    %% ==============
    %  States
    %  x = [s; vx; vy; wz; ye; theta_e; SoC]
    % ==============
    s       = SX.sym('s');        % progress along track
    vx      = SX.sym('vx');
    vy      = SX.sym('vy');
    wz      = SX.sym('wz');
    ye      = SX.sym('ye');
    theta_e = SX.sym('theta_e');
    SoC     = SX.sym('SoC');

    x  = [s; vx; vy; wz; ye; theta_e; SoC];
    nx = length(x);

    %% ==============
    %  State derivative symbol (for implicit form)
    % ==============
    xdot = SX.sym('xdot', nx, 1);

    %% ==============
    %  Inputs
    %  u = [delta; a]
    % ==============
    delta = SX.sym('delta');   % steering angle
    a     = SX.sym('a');       % longitudinal acceleration

    u  = [delta; a];
    nu = length(u);

    %% ==============
    %  Parameters
    %  p = [kappa_r; delta_prev; a_prev]
    % ==============
    p           = SX.sym('p', 7, 1);
    kappa_r     = p(1);    % reference curvature
    delta_prev  = p(2);    
    a_prev      = p(3);    

    %% ==============
    %  Vehicle parameters
    % ==============
    m   = 1.8;      % mass [kg]
    Iz  = 0.03;     % yaw inertia [kg*m^2]
    lf  = 0.125;    % CG to front axle [m]
    lr  = 0.125;    % CG to rear axle [m]
    Cf  = 68;       % front cornering stiffness [N/rad]
    Cr  = 71;       % rear cornering stiffness [N/rad]
    mu_tyre = 1.2;
    g    = 9.81;

    % Battery / aero parameters (illustrative)
    Cd      = 0.35;
    rho     = 1.225;
    Ar      = 0.05;
    C_alpha = 3000;

    %% ==============
    %  Nonlinear dynamics
    % ==============

    % Avoid division by zero (if vx ~ 0)
    eps_vx  = 1e-3;
    vx_safe = vx + eps_vx;

    % Tire lateral forces (bicycle model)
    Fy_f = Cf * ( delta - vy/vx_safe + lf*wz/vx_safe );
    Fy_r = Cr * ( -vy/vx_safe - lr*wz/vx_safe );

    % Longitudinal, lateral, yaw dynamics
    vx_dot = a ...
             - Fy_f * sin(delta) / m ...
             - mu_tyre * g ...
             + wz * vy;

    vy_dot = Fy_f * cos(delta) / m ...
             + Fy_r / m ...
             - wz * vx;

    wz_dot = ( Fy_f * lf * cos(delta) - Fy_r * lr ) / Iz;

    % Progress s-dot in curvilinear frame
    s_dot = (vx * cos(theta_e) - vy * sin(theta_e)) / (1 - ye * kappa_r);

    % Lateral error & heading error in curvilinear coordinates
    ye_dot = vx * sin(theta_e) + vy * cos(theta_e);

    theta_e_dot = wz ...
        - ( (vx * cos(theta_e) - vy * sin(theta_e)) * kappa_r ) ...
          / ( 1 - ye * kappa_r );

    % Battery power and SoC dynamics
    p_alpha = 0.5 * Cd * rho * Ar * (vx^2) + mu_tyre * m * g * vx;
    SoC_dot = -(1 / C_alpha) * p_alpha;

    % Explicit dynamics
    f_expl = [ s_dot;
               vx_dot;
               vy_dot;
               wz_dot;
               ye_dot;
               theta_e_dot;
               SoC_dot ];

    % Implicit form: xdot - f(x,u,p) = 0
    f_impl = xdot - f_expl;

    %% ==============
    %  Fill ACADOS model struct
    % ==============
    model.x           = x;
    model.xdot        = xdot;
    model.u           = u;
    model.p           = p;          
    model.f_expl_expr = f_expl;
    model.f_impl_expr = f_impl;
end
