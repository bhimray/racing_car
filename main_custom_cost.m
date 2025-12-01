%% main_racecar_acados.m
clear all; close all; clc;
import casadi.*

%% ==============
%  Load model
% ==============
model = race_car_model_mod();   % must be the updated version with s in the state

nx = length(model.x);   % 7 states: [s; vx; vy; wz; ye; theta_e; SoC]
nu = length(model.u);   % 2 inputs: [delta; a]

%% ==============
%  Load track and build curvature interpolant (like the example)
% ===============
%% Load track from LMS_Track.txt
track_file = 'LMS_Track.txt';
track_data = load(track_file);   % [s, x, y, psi, kappa_raw]

Sref       = track_data(:,1);    % arclength s
Xref       = track_data(:,2);
Yref       = track_data(:,3);
Psiref     = track_data(:,4);
kappa_raw  = track_data(:,5);

pathlength = Sref(end);

% --- scale curvature to realistic magnitude ---
scale_factor = 10;
kappa_scaled = kappa_raw / scale_factor;

% --- smooth curvature a bit to remove hard steps ---
% (window 5–7 is usually enough; can tune)
kappa_smooth = movmean(kappa_scaled, 7);

% --- make periodic extension for interpolation ---
s_ext      = [Sref; Sref(end) + Sref(2:end)];
kappa_ext  = [kappa_smooth; kappa_smooth(2:end)];

kapparef_s = interpolant('kapparef_s', 'linear', {s_ext}, kappa_ext);
[Ux_final, Ux_steady, Ux_forward, lap_time_min] = velocity_profile_gen( ...
    struct('station', Sref, 'curvature', full(kapparef_s(Sref)), 'total_length', pathlength), ...
    1.2, 1.8, 20.0);

% % Ensure column vectors
% s0       = s0(:);
% kapparef = kapparef(:);

% Build bspline interpolant: kapparef_s(s) ≈ κ(s)
% kapparef_s = interpolant('kapparef_s', 'bspline', {s0}, kapparef);
%% =============================
%  Horizon settings
% =============================
N  = 50;          % prediction horizon steps
T  = 1.0;         % horizon duration
dt = T/N;


%% =============================
%  ACADOS Model
% =============================
ocp_model = acados_ocp_model();
ocp_model.set('name', model.name);
ocp_model.set('T', T);

ocp_model.set('sym_x',    model.x);
ocp_model.set('sym_xdot', model.xdot);
ocp_model.set('sym_u',    model.u);
ocp_model.set('sym_p',    model.p);

ocp_model.set('dyn_type','explicit');
ocp_model.set('dyn_expr_f', model.f_expl_expr);


%% =============================
%  Constraints
% =============================
delta_min = -0.6;
delta_max = 0.6;

% a_max = 4.0;
% a_min = -4.0;

% Lateral bounds (track width ~ 0.24)
ye_min = -0.12;
ye_max =  0.12;

% --- state constraint on ye = x(5)
nbx = 1;
Jbx = zeros(nbx,nx);
Jbx(1,5) = 1;

ocp_model.set('constr_Jbx', Jbx);
ocp_model.set('constr_lbx', ye_min);
ocp_model.set('constr_ubx', ye_max);

% --- input bounds ---
nbu = 1;
Jbu = zeros(nbu,nu);
Jbu(1,1) = 1;   % delta
% Jbu(2,2) = 1;   % a

ocp_model.set('constr_Jbu', Jbu);
ocp_model.set('constr_lbu', delta_min);
ocp_model.set('constr_ubu', delta_max);


%% =============================
%  Initial condition
% =============================
x0 = [0.0;    % s
      0.5;    % vx
      0.0;    % vy
      0.0;    % wz
      0.0;    % ye
      0.0;    % theta_e
      95.0];  % SoC


ocp_model.set('constr_x0', x0);

u_prev = [0.0; 0.0];   % for Δu cost

%% =============================
%  Cost (MPC-RG style)
% =============================
ocp_model.set('cost_type','ext_cost');
ocp_model.set('cost_type_e','ext_cost');

x = model.x;
u = model.u;
p           = model.p;
kappa_r     = p(1);
u_prev      = p(2:3);
vx_ref      = p(4);
ye_ref      = p(5);
theta_ref   = p(6);
SoC_ref     = p(7);

e_vx    = x(2) - vx_ref;
e_ye    = x(5) - ye_ref;
e_theta = x(6) - theta_ref;
e_soc   = x(7) - SoC_ref;

% --- State weights (all non-negative) ---
Q_c = diag([ ...
    1e-8;   % s
    1;    % vx  (we want to track high vx)
    1e-2;   % vy
    1e-2;   % wz
    1e-8;    % ye
    5e-3;    % theta_e
    5e-1]);  % SoC

% no linear term for now
q_c = zeros(nx,1);

% --- Δu weights (R_c positive definite) ---
R_c = diag([
    1e-3;   % Δdelta
    1e-3]); % Δa (increase this if accel still jumps)

% --- Terminal weight (P_c) ---
P_c = diag([ ...
    1e-1;   % s
    2.0;    % vx
    1e-2;   % vy
    1e-2;   % wz
    1e-1;   % ye
    1e-1;   % theta_e
    0.01]);  % SoC

% delta u cost
Delta_u = u - u_prev;

% tracking error cost
track_err = e_vx^2 * 5.0 ...
          + e_ye^2 * 5 ...
          + e_theta^2 * 5 ...
          + e_soc^2 * 0.1;

stage_cost = track_err + Delta_u.' * R_c * Delta_u;
terminal_cost = track_err;   % or scaled

ocp_model.set('cost_expr_ext_cost', stage_cost);
ocp_model.set('cost_expr_ext_cost_e', terminal_cost);

%% =============================
%  ACADOS Options
% =============================
ocp_opts = acados_ocp_opts();
ocp_opts.set('param_scheme_N', N);
ocp_opts.set('nlp_solver', 'sqp');          % or 'sqp' sqp_rti
ocp_opts.set('nlp_solver_exact_hessian', 'false');
ocp_opts.set('sim_method', 'erk');          % explicit RK
ocp_opts.set('sim_method_num_stages', 4);
ocp_opts.set('sim_method_num_steps', 3);
ocp_opts.set('qp_solver', 'partial_condensing_hpipm');
ocp_opts.set('qp_solver_cond_N', 10);
ocp_opts.set('nlp_solver_tol_stat', 1e-4);
ocp_opts.set('nlp_solver_tol_eq',   1e-4);
ocp_opts.set('nlp_solver_tol_ineq', 1e-4);
ocp_opts.set('nlp_solver_tol_comp', 1e-4);


%% ==============
%  Create OCP solver
% ==============
ocp_solver = acados_ocp(ocp_model, ocp_opts);

%% ==============
%  Simulation setup
% ==============
Tf   = 10.0;                    % total closed-loop simulation time [s]
Nsim = round(Tf / dt);
sref_N = 3;  % reference for final reference progress

simX = zeros(Nsim, nx);
simU = zeros(Nsim, nu);
t_comp_sum = 0;
t_comp_max = 0;

x_curr = x0;
s0 = x0(1);
% Initial guesses
ocp_solver.set('init_x', repmat(x0, 1, N+1));
ocp_solver.set('init_u', zeros(nu, N));

t_mpc_list = zeros(Nsim,1);
t_mpc_max  = 0;
t_mpc_sum  = 0;
lap_time = 0;

%% ==============
%  Closed-loop MPC simulation with curvature from track
% ==============
for i = 1:Nsim

    % update reference
    sref = s0 + sref_N;
    for j = 0:(N-1)
        s_curr = s0 + (sref - s0) * j / N;

        kappa_val = full(kapparef_s(s_curr));
        vx_ref_val = Ux_final(i+j);       % desired speed
        ye_ref_val = 0.0;
        theta_ref_val = 0.0;
        SoC_ref_val = 95.0;

        p_val = [kappa_val; 0; 0; vx_ref_val; ye_ref_val; theta_ref_val; SoC_ref_val];
        ocp_solver.set('p', p_val, j);



        % [ye_min_s, ye_max_s] = track_bounds_with_obstacle( ...
        % s_curr, track_width, s_obs, ye_obs, obs_w, obs_L);

        % if j == 0
        %     lbx_full = -inf(nx,1);
        %     ubx_full =  inf(nx,1);

        %     lbx_full(nx) = ye_min_s;   % MATLAB → 1-based
        %     ubx_full(nx) = ye_max_s;

        %     ocp_solver.set('constr_lbx', lbx_full, j);
        %     ocp_solver.set('constr_ubx', ubx_full, j);
        % else
        %     ocp_solver.set('constr_lbx', ye_min_s, j);
        %     ocp_solver.set('constr_ubx', ye_max_s, j);
        % end
    end
    % yref_N = [sref, 0, 0, 0, 0, 0, 0];
    % ocp_solver.set('cost_y_ref_e', yref_N);

    t_start = tic;
    % --- update x0 constraint ---
    ocp_solver.set('constr_x0', x_curr);
    t_mpc = toc(t_start);

    t_mpc_list(i) = t_mpc;
    t_mpc_sum     = t_mpc_sum + t_mpc;
    t_mpc_max     = max(t_mpc_max, t_mpc);

    disp("x_curr:");
    disp(x_curr);
    % --- solve OCP ---
    ocp_solver.solve();
    status = ocp_solver.get('status');
    ocp_solver.print('stat'); 
    if status ~= 0 && status ~= 2
        % borrowed from acados/utils/types.h
        %statuses = {
        %    0: 'ACADOS_SUCCESS',
        %    1: 'ACADOS_NAN_DETECTED',
        %    2: 'ACADOS_MAXITER',
        %    3: 'ACADOS_MINSTEP',
        %    4: 'ACADOS_QP_FAILURE',
        %    5: 'ACADOS_READY'
        error('acados returned status %d at simulation step %d', status, i);
    end

    % --- get optimal control and state ---
    u0       = ocp_solver.get('u', 0);
    x0_stage = ocp_solver.get('x', 0);

    simX(i,:) = x0_stage';
    simU(i,:) = u0';
    x_next = ocp_solver.get('x', 1);

    % --- get lap time ----
    s        = x_curr(1);
    vx       = x_curr(2);
    vy       = x_curr(3);
    wz       = x_curr(4);
    ye       = x_curr(5);
    theta_e  = x_curr(6);
    
    x_curr = x_next;

    s_next = x_curr(1);
    s_dot = (vx*cos(theta_e) - vy*sin(theta_e)) / (1 - ye*kapparef_s(s_curr));
    dt_lap    = 1 / s_dot * (s_next - s0);   % more precise
    lap_time = lap_time + dt_lap;

    s0 = x_curr(1);

    % check if one lap is done and break and remove entries beyond
    if s0 > Sref(end) + 0.1
        % find where vehicle first crosses start line
        N0 = find(diff(sign(simX(:, 1))));
        N0 = N0(1);
        Nsim = i - N0 + 1;  % correct to final number of simulation steps for plotting
        simX = simX(N0:i, :);
        simU = simU(N0:i, :);
        break
    end
end 

%% ==============
%  Plots
% ==============
t = (0:Nsim-1) * dt;

figure;
subplot(3,1,1);
plot(t, simX(:,2)); grid on;
xlabel('t [s]'); ylabel('v_x [m/s]');
title('Longitudinal speed');

subplot(3,1,2);
plot(t, simX(:,5)); hold on;
plot(t, simX(:,6));
grid on;
xlabel('t [s]'); ylabel('ye, theta_e');
legend('ye','theta_e');
title('Track error');

subplot(3,1,3);
plot(t, simX(:,7));
grid on;
xlabel('t [s]'); ylabel('SoC');
title('State of Charge');

figure;
plot(t, simU(:,1)); hold on;
plot(t, simU(:,2));
grid on;
xlabel('t [s]'); ylabel('Inputs');
legend('\delta','a');
title('Control inputs');

% If you have the helper from the example, you can also plot on the track:
plotTrackProjection(simX, track_file);
