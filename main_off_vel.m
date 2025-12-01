%% main_racecar_acados.m
clear all; close all; clc;
import casadi.*

%% ==============
%  Load model
% ===============
model = racecar_model();   % must be the updated version with s in the state

nx = length(model.x);   % 7 states: [s; vx; vy; wz; ye; theta_e; SoC]
nu = length(model.u);   % 2 inputs: [delta; a]

%% ==============
%  Load track and build curvature interpolant (like the example)
% ==============
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

kapparef_s = interpolant('kapparef_s', 'bspline', {s_ext}, kappa_ext);

disp(full(kapparef_s(Sref)));
[Ux_final, Ux_steady, Ux_forward, lap_time_min] = velocity_profile_gen( ...
    struct('station', Sref, 'curvature', full(kapparef_s(Sref)), 'total_length', pathlength), ...
    1.2, 1.8, 20.0);

%% ==============
%  Horizon parameters
% ==============
N = 50;                 % prediction horizon steps
T = 1.0;                % total horizon time [s]
dt = T / N;


%% ==============
%  Create OCP model
% ==============
ocp_model = acados_ocp_model();
ocp_model.set('name', model.name);
ocp_model.set('T', T);

% symbolics
ocp_model.set('sym_x',    model.x);
ocp_model.set('sym_xdot', model.xdot);
ocp_model.set('sym_u',    model.u);
ocp_model.set('sym_p',    model.p);      % kappa_r as parameter

% dynamics: explicit
ocp_model.set('dyn_type', 'explicit');
ocp_model.set('dyn_expr_f', model.f_expl_expr);


%% ==============
%  Constraints
% ==============
% Your constraints:
delta_max = 0.6;
delta_min = -0.6;

% a_max = 30.0;
% a_min = -30.0;

%ye_max is constant because there is no obstacle
ye_max = 0.12;
ye_min = -0.12;

% Box constraint on ye (state index: [s=1, vx=2, vy=3, wz=4, ye=5, theta_e=6, SoC=7])
nbx = 1;
Jbx = zeros(nbx, nx);
Jbx(1,5) = 1;        % ye is x(5)

ocp_model.set('constr_Jbx',  Jbx);
ocp_model.set('constr_lbx',  ye_min);
ocp_model.set('constr_ubx',  ye_max);

% ocp_model.set('constr_expr_h', model.constr_expr_h);
% ocp_model.set('constr_lh', -inf(model.constr_nh,1));  % lower bound
% ocp_model.set('constr_uh', zeros(model.constr_nh,1)); % upper bound: h ≤ 0

% Box constraints on inputs: delta and a
nbu = 1;               % number of constrained inputs
Jbu = zeros(nbu, nu);  % nu = 2

% constrain delta → first input
Jbu(1,1) = 1;

% constrain acceleration → second input
% Jbu(2,2) = 1;

ocp_model.set('constr_Jbu', Jbu);
ocp_model.set('constr_lbu', delta_min);
ocp_model.set('constr_ubu', delta_max);

%% ==============
%  Initial condition
% ==============
% x = [s; vx; vy; wz; ye; theta_e; SoC]
s_init      = 0.0;   % start at beginning of track
vx_init     = 1.5;
vy_init     = 0.0;
wz_init     = 0.0;
ye_init     = 0.0;
theta_e_init= 0.0;
SoC_init    = 95.0;

x0 = [s_init;
      vx_init;
      vy_init;
      wz_init;
      ye_init;
      theta_e_init;
      SoC_init];

s_obs   = 5.0;
ye_obs  = 0.05;
obs_w   = 0.05;
obs_L   = 0.05;
track_width = 0.24;

ocp_model.set('constr_x0', x0);

%% ==============
%  Cost (MPC-RG style flavour: reward vx and SoC, penalize ye, theta_e, inputs)
% ===============
ocp_model.set('cost_type',   'linear_ls');
ocp_model.set('cost_type_e', 'linear_ls');

ny   = nx + nu;   % y = [x; u]
ny_e = nx;

% y = [x; u] = Vx * x + Vu * u
Vx    = zeros(ny,   nx);
Vx_e  = zeros(ny_e, nx);
Vu    = zeros(ny,   nu);

Vx(1:nx, :)   = eye(nx);
Vx_e(1:nx, :) = eye(nx);
Vu(nx+1:end,:)= eye(nu);

ocp_model.set('cost_Vx',   Vx);
ocp_model.set('cost_Vx_e', Vx_e);
ocp_model.set('cost_Vu',   Vu);

% State weights (roughly: Qc)
Q = diag([ ...
    1e-8;   % s
    1;    % vx  (we want to track high vx)
    1e-2;   % vy
    1e-2;   % wz
    1e-8;    % ye
    5e-3;    % theta_e
    5e-1]);  % SoC

% Input weights (Rc)
R = diag([ ...
    1e-3;   % delta
    1e-1]); % a

% Terminal weights (Pc)
Qe = diag([ ...
    1e-8;   % s
    1;    % vx
    1e-2;   % vy
    1e-2;   % wz
    1.0;   % ye
    1.0;   % theta_e
    0.01]);  % SoC

unscale = N/T;
W   = unscale * blkdiag(Q, R);
W_e = Qe / unscale;

ocp_model.set('cost_W',   W);
ocp_model.set('cost_W_e', W_e);

% References (x_ref, u_ref)
vx_ref    = 0.0;     % desired speed
SoC_ref   = 95.0;    % keep SoC high
ye_ref    = 0.0;
theta_ref = 0.0;

% Stage reference y_ref = [x_ref; u_ref]
y_ref   = zeros(ny,1);
y_ref_e = zeros(ny_e,1);

% x_ref:
y_ref(1) = 0.0;         % s_ref (not used)
y_ref(2) = vx_ref;
y_ref(3) = 0.0;
y_ref(4) = 0.0;
y_ref(5) = ye_ref;
y_ref(6) = theta_ref;
y_ref(7) = SoC_ref;

% u_ref:
y_ref(8) = 0.0;         % delta_ref
y_ref(9) = 0.0;         % a_ref

% terminal x_ref:
y_ref_e(1) = 0.0;
y_ref_e(2) = vx_ref;
y_ref_e(3) = 0.0;
y_ref_e(4) = 0.0;
y_ref_e(5) = ye_ref;
y_ref_e(6) = theta_ref;
y_ref_e(7) = SoC_ref;

ocp_model.set('cost_y_ref',   y_ref);
ocp_model.set('cost_y_ref_e', y_ref_e);

%% ==============
%  OCP options
% ==============
ocp_opts = acados_ocp_opts();
ocp_opts.set('param_scheme_N', N);
ocp_opts.set('nlp_solver', 'sqp');          % or 'sqp' sqp-rti
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
ocp_opts.set('regularize_method', 'convexify');

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

x_curr = x0;
s0 = x0(1);
% Initial guesses
ocp_solver.set('init_x', repmat(x0, 1, N+1));
ocp_solver.set('init_u', zeros(nu, N));

%% ==============
%  Closed-loop MPC simulation with curvature from track
% ==============

t_mpc_list = zeros(Nsim,1);
t_mpc_max  = 0;
t_mpc_sum  = 0;
lap_time = 0;

for i = 1:Nsim

    % update reference
    sref = s0 + sref_N;
    for j = 0:(N-1)
        s_curr = s0 + (sref - s0) * j / N;
        yref = [0, Ux_final(i+j), 0, 0, 0, 0, 0, 0, 0];

        ocp_solver.set('cost_y_ref', yref, j);
        kappa_val = full(kapparef_s(s_curr));
        ocp_solver.set('p', kappa_val, j);

        fprintf('Curvature at stage 0 = %.6f\n', ocp_solver.get('p',0));

        [ye_min_s, ye_max_s] = track_bounds_with_obstacle( ...
        s_curr, track_width, s_obs, ye_obs, obs_w, obs_L);
        if j == 0
            lbx_full = -inf(nx,1);
            ubx_full =  inf(nx,1);

            lbx_full(nx) = ye_min_s;   % MATLAB → 1-based
            ubx_full(nx) = ye_max_s;

            ocp_solver.set('constr_lbx', lbx_full, j);
            ocp_solver.set('constr_ubx', ubx_full, j);
        else
            ocp_solver.set('constr_lbx', ye_min_s, j);
            ocp_solver.set('constr_ubx', ye_max_s, j);
        end
    end
    yref_N = [sref, 0, 0, 0, 0, 0, 0];
    ocp_solver.set('cost_y_ref_e', yref_N);

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

t_mpc_avg = t_mpc_sum / Nsim;
Ts        = dt;       % sampling time
CI_avg    = t_mpc_avg / Ts;
CI_max    = t_mpc_max / Ts;

fprintf('\n===== MPC Timing Report =====\n');
fprintf('Average MPC solve time  : %.6f s\n', t_mpc_avg);
fprintf('Maximum MPC solve time  : %.6f s\n', t_mpc_max);
fprintf('Sampling time Ts        : %.6f s\n', Ts);
fprintf('CI_avg = Tc_avg/Ts      : %.4f\n', CI_avg);
fprintf('CI_max = Tc_max/Ts      : %.4f\n', CI_max);

if CI_max < 1
    fprintf('Status: REAL-TIME OK (CI_max < 1)\n');
else
    fprintf('Status: NOT REAL-TIME (CI_max >= 1)\n');
end


disp("laptime");
disp(lap_time);

disp("velocity profile plot");
disp(size(Ux_final));
disp(size(Ux_steady));
disp(size(Ux_forward));
disp(lap_time_min);

%% ==============
%  Plots
% ==============
t = (0:Nsim-1) * dt;

plot_velocity_profile( ...
    struct('station', Sref, 'curvature', full(kapparef_s(Sref)), 'total_length', pathlength), ...
    Ux_final, Ux_steady, Ux_forward);

figure;
subplot(3,1,1);
plot(t, simX(:,2)); grid on;hold on;
plot(t, simX(:,3));
plot(t, simX(:,4));
xlabel('t [s]'); ylabel('v_x, v_y [m/s], wz [rad/s]');
legend('v_x','v_y','w_z');
title('Speed');

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
[Xtraj, Ytraj ] = plotTrackProjection(simX, track_file);
hold on;
plotObstacle(track_file, s_obs, ye_obs, obs_w, obs_L);

Accel = simU(:,2);  % longitudinal acceleration
figure;
scatter(Xtraj, Ytraj, 30, Accel, 'filled');
colorbar;
colormap(jet);
title('Acceleration along the track');
xlabel('X [m]'); ylabel('Y [m]');
axis equal;
grid on;


