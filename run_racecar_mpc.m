function run_racecar_mpc()

    ocp = racecar_ocp_setup();

    N = 15;         % same as in setup
    nx = 6;
    nu = 2;

    x_current = [1.0; 0; 0; 0; 0; 80];   % initial state

    for k = 1:200

        % Example curvature profile (replace with track)
        kappa_profile = 0.02 * ones(N,1);

        % Set parameters for all shooting nodes
        for i = 0:N-1
            ocp.set('p', kappa_profile(i+1), i);
        end

        % Set initial state
        ocp.set('x', x_current, 0);

        % Solve OCP
        ocp.solve();

        % Extract optimal control
        u0 = ocp.get('u', 0);

        % Simulate one step
        x_next = simulate_racecar_step(x_current, u0);

        x_current = x_next;

        fprintf('Step %03d | vx = %.3f | ye = %.3f | delta = %.3f\n', ...
                k, x_current(1), x_current(4), u0(1));

    end

end
