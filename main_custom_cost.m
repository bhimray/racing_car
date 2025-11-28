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
delta_max = 0.6;
delta_min = -0.6;

a_max = 4.0;
a_min = -4.0;

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
nbu = 2;
Jbu = zeros(nbu,nu);
Jbu(1,1) = 1;   % delta
Jbu(2,2) = 1;   % a

ocp_model.set('constr_Jbu', Jbu);
ocp_model.set('constr_lbu', [delta_min; a_min]);
ocp_model.set('constr_ubu', [delta_max; a_max]);


%% =============================
%  Initial condition
% =============================
x0 = [0.0;    % s
      1.5;    % vx
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
p = model.p;         % p = [kappa_r; delta_prev; a_prev]

u_prev_param = p(2:3);     % previous inputs from p
Delta_u = u - u_prev_param;


% --- State weights (all non-negative) ---
Q_c = diag([ ...
    1e-1;   % s
    1e-8;    % vx  (we want to track high vx)
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
    1e-2]); % Δa (increase this if accel still jumps)

% --- Terminal weight (P_c) ---
P_c = diag([ ...
    1e-1;   % s
    2.0;    % vx
    1e-2;   % vy
    1e-2;   % wz
    1e-1;   % ye
    1e-1;   % theta_e
    0.01]);  % SoC

stage_cost = x.'*Q_c*x + q_c.'*x + Delta_u.'*R_c*Delta_u;
terminal_cost = x.'*P_c*x;

ocp_model.set('cost_expr_ext_cost', stage_cost);
ocp_model.set('cost_expr_ext_cost_e', terminal_cost);


%% =============================
%  ACADOS Options
% =============================
ocp_opts = acados_ocp_opts();
ocp_opts.set('param_scheme_N', N);
ocp_opts.set('nlp_solver', 'sqp');
ocp_opts.set('nlp_solver_exact_hessian','false');
ocp_opts.set('sim_method','erk');
ocp_opts.set('qp_solver','partial_condensing_hpipm');
ocp_opts.set('qp_solver_cond_N', 10);


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

%% ==============
%  Closed-loop MPC simulation with curvature from track
% ==============
for i = 1:Nsim

    % update reference
    sref = s0 + sref_N;
    for j = 0:(N-1)
        disp("X");
        disp(x_curr)
        s_curr = s0 + (sref - s0) * j / N;
        kappa_val = full(kapparef_s(s_curr));
        p_val = [kappa_val; u_prev(:)];
        ocp_solver.set('p', p_val, j);
        % fprintf('Curvature at stage 0 = %.6f\n', ocp_solver.get('p',0));
    end

    % --- update x0 constraint ---
    ocp_solver.set('constr_x0', x_curr);

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

    % propagate to next step using predicted state at stage 1
    x_next = ocp_solver.get('x', 1);
    x_curr = x_next;
    s0 = x_curr(1);
    % update previous input for Δu at next step
    u_prev = u0;

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
