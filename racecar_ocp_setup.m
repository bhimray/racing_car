function ocp = racecar_ocp_setup()

    import casadi.*
    import acados_template.*

    %% Load nonlinear dynamic model
    model = racecar_model();       % <-- your model file
    model_name = model.name;

    %% Create OCP model
    ocp_model = acados_ocp_model();
    ocp_model.set('name', model_name);
    ocp_model.set('T', 1.5);       % Total horizon = 1.5 sec

    nx = length(model.x);
    nu = length(model.u);
    np = 1;

    %% ===== Assign dynamics (implicit form) =====
    ocp_model.set('dyn_type', 'implicit');
    ocp_model.set('dyn_expr_f', model.f_impl_expr);

    %% ===== Set dimensions =====
    ocp_model.set('nx', nx);
    ocp_model.set('nu', nu);
    ocp_model.set('np', np);

    %% ===== Constraints =====
    % Steering limits
    delta_max = 0.6;
    delta_min = -0.6;

    % Longitudinal acceleration limits
    a_max = 4;
    a_min = -4;

    % Lateral error limits
    ye_max = 0.5;
    ye_min = -0.5;

    % bounds on control inputs
    ocp_model.set('lbu', [delta_min; a_min]);
    ocp_model.set('ubu', [delta_max; a_max]);
    ocp_model.set('idxbu', [1; 2]);

    %% ===== State constraints (only ye) =====
    ocp_model.set('lh', ye_min);
    ocp_model.set('uh', ye_max);
    ocp_model.set('idxh', 4);   % ye is x(4)

    %% ===== Cost Function =====
    ocp_model.set('cost_type', 'linear_ls');
    ocp_model.set('cost_type_e', 'linear_ls');

    % Cost weights (tune based on paper)
    Q = diag([0.1, 0.2, 0.05, 2.0, 2.0, 0.05]);   % vx, vy, wz, ye, theta_e, SoC
    R = diag([2.0, 0.5]);                         % delta, a
    Qe = Q;

    ocp_model.set('W', blkdiag(Q, R));
    ocp_model.set('W_e', Qe);

    % linear cost expressions
    y_expr = [model.x; model.u];
    y_expr_e = model.x;

    ocp_model.set('cost_y_expr', y_expr);
    ocp_model.set('cost_y_expr_e', y_expr_e);

    % Reference tracking
    yref = zeros(nx + nu, 1);
    yref_e = zeros(nx, 1);
    ocp_model.set('cost_y_ref', yref);
    ocp_model.set('cost_y_ref_e', yref_e);

    %% ===== Parameter =====
    ocp_model.set('p', model.p);   % curvature kappa_r

    %% ===== Solver Options =====
    ocp_opts = acados_ocp_opts();
    ocp_opts.set('compile_interface', 'auto');
    ocp_opts.set('codgen_model', 'true');
    ocp_opts.set('nlp_solver', 'sqp');
    ocp_opts.set('nlp_solver_max_iter', 100);
    ocp_opts.set('nlp_solver_tol_stat', 1e-6);
    ocp_opts.set('nlp_solver_tol_eq', 1e-6);
    ocp_opts.set('nlp_solver_tol_comp', 1e-6);

    ocp_opts.set('sim_method', 'irk');     % implicit integrator = stable
    ocp_opts.set('sim_method_num_stages', 3);
    ocp_opts.set('sim_method_num_steps', 3);

    ocp_opts.set('qp_solver', 'partial_condensing_hpipm');
    ocp_opts.set('qp_solver_cond_N', 5);

    N = 15;                         % horizon length
    ocp_opts.set('param_scheme_N', N);
    ocp_model.set('N', N);

    %% ===== Construct the ACADOS OCP solver =====
    ocp = acados_ocp(ocp_model, ocp_opts);

    fprintf('ACADOS OCP solver for Racecar Model created successfully.\n');

end
