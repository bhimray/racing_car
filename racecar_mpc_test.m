function racecar_mpc_test()

    clc; close all;

    %% 1) Load OCP solver
    ocp = racecar_ocp_setup();

    %% Simulation parameters
    Nsim = 150;             % number of simulation steps
    Ts   = 0.1;             % sampling time (same as integrator)
    N    = 15;              % horizon

    %% State/control sizes
    nx = 6;
    nu = 2;

    %% Initial condition
    x_current = [1.0; 0; 0; 0; 0; 90];     % [vx vy wz ye theta_e SoC]

    %% Logging
    X_log = zeros(nx, Nsim+1);
    U_log = zeros(nu, Nsim);
    solver_time = zeros(Nsim,1);
    status_log = zeros(Nsim,1);

    X_log(:,1) = x_current;

    %% Warm-start memory
    u_prev = zeros(nu, N);
    x_prev = repmat(x_current, 1, N+1);

    %% Track curvature profile (flat)
    kappa_profile = 0.02 * ones(N,1);

    %% Prepare figures
    figure(1); clf;
    tiledlayout(3,2);

    %% Simulation Loop
    for k = 1:Nsim

        %% 2) Set parameters (curvature)
        for i = 0:N-1
            ocp.set('p', kappa_profile(i+1), i);
        end

        %% 3) Warm-start previous solution
        for i = 0:N-1
            ocp.set('x', x_prev(:,i+1), i);
            ocp.set('u', u_prev(:,i+1), i);
        end
        ocp.set('x', x_prev(:,N+1), N);

        %% 4) Clamp the current state at node 0
        ocp.set('x', x_current, 0);

        %% 5) Solve MPC
        tic
        ocp.solve();
        solver_time(k) = toc;

        status = ocp.get('status');
        status_log(k) = status;

        if status ~= 0
            fprintf('Solver failed at step %d with status %d\n', k, status);
            break;
        end

        %% 6) Extract optimal control
        u0 = ocp.get('u', 0);
        U_log(:,k) = u0;

        %% 7) Extract predicted solution for warm-start
        for i = 0:N-1
            x_prev(:,i+1) = ocp.get('x', i);
            u_prev(:,i+1) = ocp.get('u', i);
        end
        x_prev(:,N+1) = ocp.get('x', N);

        %% 8) Simulate forward 1 step
        x_next = simulate_racecar_step(x_current, u0);
        x_current = x_next;
        X_log(:,k+1) = x_current;

        %% -----------------------
        %   LIVE PLOTS
        % -----------------------
        nexttile(1);
        plot(0:k, X_log(1,1:k+1), 'b'); grid on; title('vx (m/s)');

        nexttile(2);
        plot(0:k, X_log(4,1:k+1), 'r'); grid on; title('ye (m)');

        nexttile(3);
        plot(0:k-1, U_log(1,1:k), 'k'); grid on; title('delta (rad)');

        nexttile(4);
        plot(0:k-1, U_log(2,1:k), 'm'); grid on; title('a (m/s^2)');

        nexttile(5);
        plot(0:k-1, solver_time(1:k)*1000); grid on; title('Solver Time (ms)');

        nexttile(6);
        stairs(0:k-1, status_log(1:k)); grid on; title('Status');

        drawnow;

    end

    %% END loop
    fprintf("\n===== SIMULATION FINISHED =====\n");

    %% Check solver performance
    if any(status_log ~= 0)
        fprintf("⚠️ Some solver failures detected.\n");
    else
        fprintf("✔ All ACADOS solves successful.\n");
    end

end


%% ===== SIMPLE SIMULATOR =====
function x_next = simulate_racecar_step(x, u)
    model = racecar_model();
    dt = 0.1;

    f = casadi.Function('f', {model.x, model.u, model.p}, {model.f_expl_expr});
    kappa = 0;
    dx = full(f(x, u, kappa));

    x_next = x + dt * dx;
end
