function [x, y] = plotTrackProjection(simX, filename)

    s        = simX(:,1);    % progress
    ye       = simX(:,5);    % lateral error
    theta_e  = simX(:,6);    % heading error
    vx       = simX(:,2);    % for color if needed

    % Load track
    [Sref, Xref, Yref, Psiref, ~] = getTrack(filename);
    track_length = Sref(end);

    % Convert (s, ye, theta_e) -> (x, y)
    N = length(s);
    x = zeros(N,1);
    y = zeros(N,1);

    for i = 1:N
        % wrap s
        si = mod(s(i), track_length);

        % Find closest index
        idx = findClosestS(si, Sref);

        psi_ref = Psiref(idx);

        x(i) = Xref(idx) - ye(i) * sin(psi_ref);
        y(i) = Yref(idx) + ye(i) * cos(psi_ref);
    end

    %% Plot
    figure; hold on;

    % Track centerline
    plot(Xref, Yref, 'k--', 'LineWidth', 1.0);

    % Track boundaries (distance is half-width)
    distance = 0.12;
    X_L = Xref - distance * sin(Psiref);
    Y_L = Yref + distance * cos(Psiref);
    X_R = Xref + distance * sin(Psiref);
    Y_R = Yref - distance * cos(Psiref);

    plot(X_L, Y_L, 'k', 'LineWidth', 1.5);
    plot(X_R, Y_R, 'k', 'LineWidth', 1.5);

    % Vehicle path
    plot(x, y, 'b', 'LineWidth', 1.5);

    xlabel('x [m]');
    ylabel('y [m]');
    axis equal;
    grid on;

    hold off;
end
