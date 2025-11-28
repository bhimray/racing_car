function model = racecar_model()
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

    x = [s; vx; vy; wz; ye; theta_e; SoC];
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

    u = [delta; a];

    %% ==============
    %  Parameters (track curvature)
    %  p = kappa_r
    % ==============
    kappa_r = SX.sym('kappa_r');   % reference curvature
    p = kappa_r;                   % 1x1 SX

    %% ==============
    %  Vehicle parameters
    % ==============
    m   = 1.8;      % mass [kg]
    Iz  = 0.03;     % yaw inertia [kg*m^2]
    lf  = 0.125;    % CG to front axle [m]
    lr  = 0.125;    % CG to rear axle [m]
    Cf  = 68;       % front cornering stiffness [N/rad]
    Cr  = 71;       % rear cornering stiffness [N/rad]
    mu_tyre = 1.2;     % for tire friction
    mu_rr   = 0.01;    % rolling resistance for longitudinal drag     % static friction coefficient
    alpha   = 0.9;     % 
    mu_eff = mu_tyre * alpha;
    g    = 9.81;    % gravity [m/s^2]

    % Battery / aero parameters (illustrative)
    Cd      = 0.35;    % drag coefficient
    rho     = 1.225;   % air density [kg/m^3]
    Ar      = 0.05;    % frontal area [m^2]
    C_alpha = 3000;    % effective capacity scaling

    %% ==============
    %  Nonlinear dynamics
    % ==============

    % Avoid division by zero (if vx ~ 0)
    eps_vx = 1e-3;
    vx_safe = vx + eps_vx;

    % Tire lateral forces (bicycle model)
    Fy_f = Cf * ( delta - vy/vx_safe + lf*wz/vx_safe );
    Fy_r = Cr * ( -vy/vx_safe - lr*wz/vx_safe );

    % Fz_f = m * g * lr / (lf + lr);   % front normal load
    % Fz_r = m * g * lf / (lf + lr);   % rear normal load
    
    % Fy = Fy_f + Fy_r;
    % Fz = Fz_f + Fz_r;
    % F_max = m * a;
    % Fx_f = F_max * (Fz_f)/(Fz_f + Fz_r);
    % Fx_r = F_max * (Fz_r)/(Fz_f + Fz_r);
    % Fx = m * a;
    
    % % ==========================
    % % NONLINEAR FRICTION ELLIPSE
    % % ==========================
    % h_fric = (Fy/(mu_eff*Fz))^2 + (Fx/(mu_eff*Fz))^2 - 1;

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

    % Pack explicit dynamics
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
    % model.constr_expr_h = h_fric;   % nonlinear constraint expression
    % model.constr_nh = 1;            % number of nonlinear constraints

end
