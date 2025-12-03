
function plot_velocity_profile(track, Ux_final, Ux_steady, Ux_forward)

    K = track.curvature;
    s = track.station;

    figure();
    hold on;
    % Subplot (a): Sample curvature profile (paper's Fig 3a)
    subplot(2,2,1);
    plot(s, K, 'b-', 'LineWidth', 2);
    xlabel('Distance along path, s (m)');
    ylabel('Curvature, K(s) (1/m)');
    title('(a) Sample Curvature Profile');
    grid on;
    xlim([0, track.total_length]);
    
    % Subplot (b): Velocity profile given zero longitudinal force (Fig 3b)
    subplot(2,2,2);
    plot(s, Ux_steady, 'r-', 'LineWidth', 2);
    xlabel('Distance along path, s (m)');
    ylabel('Velocity (m/s)');
    title('(b) Velocity Profile (Zero Longitudinal Force)');
    grid on;
    xlim([0, track.total_length]);
    
    % Subplot (c): Velocity profile after forward pass (Fig 3c)
    subplot(2,2,3);
    plot(s, Ux_forward, 'g-', 'LineWidth', 2);
    xlabel('Distance along path, s (m)');
    ylabel('Velocity (m/s)');
    title('(c) Velocity Profile After Forward Pass');
    grid on;
    xlim([0, track.total_length]);
    
    % Subplot (d): Final velocity profile after backward pass (Fig 3d)
    subplot(2,2,4);
    plot(s, Ux_steady, 'r--', 'LineWidth', 1, 'DisplayName', 'Steady-state');
    hold on;
    plot(s, Ux_forward, 'g--', 'LineWidth', 1, 'DisplayName', 'Forward pass');
    plot(s, Ux_final, 'b-', 'LineWidth', 2, 'DisplayName', 'Final velocity');
    xlabel('Distance along path, s (m)');
    ylabel('Velocity (m/s)');
    title('(d) Final Velocity Profile After Backward Pass');
    grid on;
    xlim([0, track.total_length]);
    hold off;
    
    sgtitle('Three-Pass Velocity Profile Generation Method');
end