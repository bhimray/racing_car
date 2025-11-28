%% main_racecar_acados_new.m
clear all; close all; clc;
import casadi.*

%% =========================
%  1) Load racecar model
% =========================
% Your existing function – unchanged
mdl_struct = racecar_model();   % returns x, xdot, u, p, f_expl_expr, f_impl_expr

nx = length(mdl_struct.x);   % 7 states: [s; vx; vy; wz; ye; theta_e; SoC]
nu = length(mdl_struct.u);   % 2 inputs: [delta; a]

% Wrap into AcadosModel (new API)
model = AcadosModel();
model.name        = mdl_struct.name;
model.x           = mdl_struct.x;
model.xdot        = mdl_struct.xdot;
model.u           = mdl_struct.u;
model.p           = mdl_struct.p;          % curvature parameter kappa_r
model.f_expl_expr = mdl_struct.f_expl_expr;
model.f_impl_expr = mdl_struct.f_impl_expr;

%% =========================
%  2) Track + curvature
% =========================
track_file = 'LMS_Track.txt';
track_data = load(track_file);   % [s, x, y, psi, kappa_raw]

Sref      = track_data(:,1);
Xref      = track_data(:,2);
Yref      = track_data(:,3);
Psiref    = track_data(:,4);
kappa_raw = track_data(:,5);

pathlength = Sref(end);

% Scale + smooth curvature
scale_factor  = 10;
kappa_scaled  = kappa_raw / scale_factor;
kappa_smooth  = movmean(kappa_scaled, 7);

% periodic extension for interpolation
s_ext     = [Sref; Sref(end) + Sref(2:end)];
kappa_ext = [kappa_smooth; kappa_smooth(2:end)];

kapparef_s = interpolant('kapparef_s', 'bspline', {s_ext}, kappa_ext);

%% =========================
%  3) Horizon setup
% =========================
N  = 50;            % number of shooting intervals
T  = 1.0;           % prediction horizon [s]
dt = T / N;

%% =========================
%  4) Build AcadosOcp object
% =========================
ocp = AcadosOcp();
ocp.model = model;

% ---- dims / time grid ----
ocp.solver_options.N_horizon = N;
ocp.solver_options.tf        = T;

%% =========================
%  5) Constraints
% =========================
% input bounds
delta_max = 0.6;
delta_min = -0.6;

a_max = 6.0;
a_min = -6.0;

% lateral error bounds (track bounds BEFORE obstacle shrinking)
track_width = 0.24;
ye_max      =  0.12;
ye_min      = -0.12;

% state indices (0-based for acados)
% x = [s; vx; vy; wz; ye; theta_e; SoC]
idx_s   = 0;
idx_vx  = 1;
idx_vy  = 2;
idx_wz  = 3;
idx_ye  = 4;
idx_th  = 5;
idx_soc = 6;

% --------- hard bounds on ye, with softening via slack ----------
ocp.constraints.idxbx  = idx_ye;      % which state has bounds
ocp.constraints.lbx    = ye_min;
ocp.constraints.ubx    = ye_max;

% soft-bounds on the same state (mpc-rg-style slack)
ocp.constraints.idxsbx = 0;     % soften the only bounded state (ye)
ocp.constraints.lsbx   = 0;
ocp.constraints.usbx   = 0;


nsoft = 1;                 % only 1 slack: for ye
w_soft_L1 = 100;           % linear slack penalty
w_soft_L2 = 10;            % quadratic slack penalty

% penalties on slack variables (see Table 2 in problem_formulation_ocp_mex)
ocp.cost.zl = w_soft_L1 * ones(nsoft,1);   % lower L1
ocp.cost.zu = w_soft_L1 * ones(nsoft,1);   % upper L1
ocp.cost.Zl = w_soft_L2 * ones(nsoft,1);   % lower L2 diag
ocp.cost.Zu = w_soft_L2 * ones(nsoft,1);   % upper L2 diag

% --------- input bounds ----------
ocp.constraints.idxbu = [0, 1];                 % u = [delta; a]  (0-based)
ocp.constraints.lbu   = [delta_min; a_min];
ocp.constraints.ubu   = [delta_max; a_max];

% --------- initial condition as bound at node 0 ----------
s_init       = 0.0;
vx_init      = 2;
vy_init      = 0.0;
wz_init      = 0.0;
ye_init      = 0.0;
theta_e_init = 0.0;
SoC_init     = 95.0;

x0 = [s_init;
      vx_init;
      vy_init;
      wz_init;
      ye_init;
      theta_e_init;
      SoC_init];

% enforce x(0) = x0 using lbx_0 / ubx_0
ocp.constraints.idxbx_0 = 0:(nx-1);      % all states fixed at node 0
ocp.constraints.lbx_0   = x0;
ocp.constraints.ubx_0   = x0;

% obstacle description (same numbers as before)
s_obs   = 73.0;
ye_obs  = 0.2;
obs_w   = 0.2;
obs_L   = 0.2;

%% =========================
%  6) Cost (LINEAR_LS)
% =========================
ocp.cost.cost_type_0 = 'LINEAR_LS';
ocp.cost.cost_type   = 'LINEAR_LS';
ocp.cost.cost_type_e = 'LINEAR_LS';

ny   = nx + nu;   % y = [x; u]
ny_e = nx;

% y = [x; u] = Vx x + Vu u
Vx    = zeros(ny,   nx);
Vx_e  = zeros(ny_e, nx);
Vu    = zeros(ny,   nu);

Vx(1:nx, :)    = eye(nx);
Vx_e(1:nx, :)  = eye(nx);
Vu(nx+1:end,:) = eye(nu);

ocp.cost.Vx   = Vx;
ocp.cost.Vx_e = Vx_e;
ocp.cost.Vu   = Vu;

% ---- weights ----
Q = diag([ ...
    1e-1;   % s
    1e-8;   % vx
    1e-2;   % vy
    1e-2;   % wz
    1e-8;   % ye
    5e-3;   % theta_e
    5e-1]); % SoC

R = diag([ ...
    1e-3;   % delta
    1e-3]); % a

Qe = diag([ ...
    1e-1;   % s
    0.01;   % vx
    1e-2;   % vy
    1e-2;   % wz
    1.0;    % ye
    1.0;    % theta_e
    0.01]); % SoC

unscale = N / T;
W   = unscale * blkdiag(Q, R);
W_e = Qe / unscale;

ocp.cost.W   = W;
ocp.cost.W_e = W_e;

% ---- references ----
vx_ref    = 0.0;
SoC_ref   = 95.0;
ye_ref    = 0.0;
theta_ref = 0.0;

y_ref   = zeros(ny,1);
y_ref_e = zeros(ny_e,1);

% x_ref:
y_ref(1) = 0.0;       % s_ref (will be overwritten online)
y_ref(2) = vx_ref;
y_ref(3) = 0.0;
y_ref(4) = 0.0;
y_ref(5) = ye_ref;
y_ref(6) = theta_ref;
y_ref(7) = SoC_ref;

% u_ref:
y_ref(8) = 0.0;       % delta_ref
y_ref(9) = 0.0;       % a_ref

% terminal x_ref:
y_ref_e(1) = 0.0;
y_ref_e(2) = vx_ref;
y_ref_e(3) = 0.0;
y_ref_e(4) = 0.0;
y_ref_e(5) = ye_ref;
y_ref_e(6) = theta_ref;
y_ref_e(7) = SoC_ref;

ocp.cost.yref   = y_ref;
ocp.cost.yref_e = y_ref_e;

%% ========== INITIAL COST (required for linear_ls) ==========
ocp.cost.W_0   = W;       % same weighting as normal stage cost
ocp.cost.Vx_0  = Vx;      % maps x into y
ocp.cost.Vu_0  = Vu;      % maps u into y
ocp.cost.yref_0 = y_ref;  % reference at node 0


%% =========================
%  7) Solver options
% =========================
ocp.solver_options.integrator_type = 'ERK';     % explicit RK
ocp.solver_options.sim_method_num_stages = 4;
ocp.solver_options.sim_method_num_steps  = 3;

ocp.solver_options.nlp_solver_type        = 'SQP';
ocp.solver_options.hessian_approx         = 'GAUSS_NEWTON';
ocp.solver_options.qp_solver              = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.qp_solver_cond_N       = 10;

ocp.solver_options.nlp_solver_tol_stat    = 1e-4;
ocp.solver_options.nlp_solver_tol_eq      = 1e-4;
ocp.solver_options.nlp_solver_tol_ineq    = 1e-4;
ocp.solver_options.nlp_solver_tol_comp    = 1e-4;
ocp.solver_options.print_level            = 1;

%% =========================
%  8) Create solver
% =========================
ocp_solver = AcadosOcpSolver(ocp);

%% =========================
%  9) Closed-loop simulation
% =========================
Tf   = 10.0;
Nsim = round(Tf / dt);
sref_N = 3.0;   % progress increment reference

simX = zeros(Nsim, nx);
simU = zeros(Nsim, nu);

x_curr = x0;
s0     = x0(1);

for i = 1:Nsim

    % ---- progress reference for this MPC call ----
    sref = s0 + sref_N;

    % set stage-wise references + curvature + shrunken track bounds
    for j = 0:(N-1)
        % linearly ramp s from current s0 to sref
        s_curr = s0 + (sref - s0) * j / N;

        yref = y_ref;
        yref(1) = s_curr;      % set reference s

        % curvature as parameter
        kappa_val = full(kapparef_s(s_curr));
        ocp_solver.set('p', kappa_val, j);

        % obstacle-dependent lateral bounds
        [ye_min_s, ye_max_s] = track_bounds_with_obstacle( ...
            s_curr, track_width, s_obs, ye_obs, obs_w, obs_L);
        if j == 0
            % DO NOT SET constr_lbx OR constr_ubx at j=0
        else
            ocp_solver.set('constr_lbx', ye_min_s, j);
            ocp_solver.set('constr_ubx', ye_max_s, j);
        end
    end

    % terminal reference (only state)
    yref_e = y_ref_e;
    yref_e(1) = sref;
    ocp_solver.set('cost_y_ref_e', yref_e);

    ocp_solver.set('constr_x0', x_curr);
    disp("x");
    disp(x_curr');
    % solve
    ocp_solver.solve();
    status = ocp_solver.get('status');

    ocp_solver.print('stat');

    if status ~= 0 && status ~= 2
        error('acados returned status %d at simulation step %d', status, i);
    end

    % get optimal input + state
    u0       = ocp_solver.get('u', 0);
    x0_stage = ocp_solver.get('x', 0);

    simX(i,:) = x0_stage';
    simU(i,:) = u0';

    % propagate with state at stage 1
    x_next = ocp_solver.get('x', 1);
    x_curr = x_next;
    s0     = x_curr(1);

    % stop after one lap (same logic as before)
    if s0 > Sref(end) + 0.1
        N0   = find(diff(sign(simX(:,1))));
        N0   = N0(1);
        Nsim = i - N0 + 1;
        simX = simX(N0:i, :);
        simU = simU(N0:i, :);
        break
    end
end

%% =========================
%  10) Plots
% =========================
t = (0:Nsim-1) * dt;

figure;
subplot(3,1,1);
plot(t, simX(:,2)); hold on;
plot(t, simX(:,3));
plot(t, simX(:,4));
grid on;
xlabel('t [s]'); ylabel('v_x, v_y [m/s], w_z [rad/s]');
legend('v_x','v_y','w_z');
title('Speed states');

subplot(3,1,2);
plot(t, simX(:,5)); hold on;
plot(t, simX(:,6));
grid on;
xlabel('t [s]'); ylabel('y_e, \theta_e');
legend('y_e','\theta_e');
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

% trajectory colored by acceleration
[Xtraj, Ytraj] = plotTrackProjection(simX, track_file);   % make sure this returns X,Y
Accel = simU(:,2);
figure;
scatter(Xtraj, Ytraj, 30, Accel, 'filled');
colorbar;
colormap(jet);
title('Acceleration along the track');
xlabel('X [m]'); ylabel('Y [m]');
axis equal;
grid on;
