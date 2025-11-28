%% ============================================================
%  CHECK FRICTION ELLIPSE FEASIBILITY FOR YOUR RACING MPC
% =============================================================

clear; clc;

%% =====================
%  VEHICLE PARAMETERS
% ======================
m   = 1.8;
g   = 9.81;

lf  = 0.125;      % CG to front axle
lr  = 0.125;      % CG to rear axle
Cf  = 68;         % front cornering stiffness
Cr  = 71;         % rear cornering stiffness

% Effective friction coefficient (for ellipse)
mu_tyre = 1.0;    % physical dry grip
alpha   = 0.9;    % tightening factor
mu_eff  = mu_tyre * alpha;

%% =====================
%  INITIAL STATE x0
% ======================
vx      = 1.5;
vy      = 0.0;
wz      = 0.0;
delta0  = 0.0;
a0      = 0.0;

vx_safe = vx + 1e-3;

%% =====================
%  INPUT BOUNDS — YOU MUST SET THESE
% ======================
delta_min = -0.5;     % rad
delta_max =  0.5;

a_min = -5;           % m/s^2
a_max =  5;

% Create grid
delta_vals = linspace(delta_min, delta_max, 51);
a_vals     = linspace(a_min, a_max, 51);

%% =====================
%  PRECOMPUTE NORMAL LOADS
% ======================
Fz_f = m*g*lr/(lf+lr);
Fz_r = m*g*lf/(lf+lr);
Fz   = Fz_f + Fz_r;

%% =====================
%  FEASIBILITY CHECK
% ======================
feasible = false;
count_feasible = 0;
count_total = length(delta_vals) * length(a_vals);

% Storage for plotting
D = [];
A = [];

for d = delta_vals
    for ax = a_vals

        %% Lateral forces (bicycle model)
        Fy_f = Cf * ( d - vy/vx_safe + lf*wz/vx_safe );
        Fy_r = Cr * ( -vy/vx_safe - lr*wz/vx_safe );
        Fy   = Fy_f + Fy_r;

        %% Longitudinal forces from acceleration input
        Fx_total = m * ax;

        % Distribute according to normal loads (50/50)
        Fx_f = Fx_total * (Fz_f/(Fz_f + Fz_r));
        Fx_r = Fx_total * (Fz_r/(Fz_f + Fz_r));
        Fx   = Fx_f + Fx_r;

        %% FRICTION ELLIPSE h(x,u)
        h_fric = (Fy/(mu_eff*Fz))^2 + (Fx/(mu_eff*Fz))^2 - 1;

        if h_fric <= 0
            feasible = true;
            count_feasible = count_feasible + 1;
            D(end+1) = d;
            A(end+1) = ax;
        end

    end
end

%% =====================
%  PRINT RESULTS
% ======================
fprintf("Friction ellipse feasible ANYWHERE?  %d\n", feasible);
fprintf("Feasible combos: %d / %d (%.2f%%)\n", ...
    count_feasible, count_total, 100*count_feasible/count_total);

%% =====================
%  PLOT FEASIBLE REGION
% ======================
figure; hold on;
scatter(D, A, 12, 'filled');
xlabel('\delta (rad)');
ylabel('a (m/s^2)');
title('Feasible region under friction ellipse');
grid on;
