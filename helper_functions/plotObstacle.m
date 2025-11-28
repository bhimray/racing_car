function plotObstacle(track_file, s_obs, ye_obs, obs_w, obs_L)
    % track_file : LMS_Track.txt
    % s_obs      : obstacle position on centerline (meters)
    % ye_obs     : lateral offset (meters)
    % obs_w      : width of obstacle (left–right)
    % obs_L      : length of obstacle (front–back)

    % ---- load track ----
    data = load(track_file);
    Sref = data(:,1);
    Xref = data(:,2);
    Yref = data(:,3);
    Psiref = data(:,4);

    % ---- convert obstacle center (s_obs, ye_obs) to (X,Y) ----
    % find closest index on centerline
    [~, idx] = min(abs(Sref - s_obs));

    xc = Xref(idx);
    yc = Yref(idx);
    psi = Psiref(idx);     % centerline orientation

    % convert ye offset into global coordinates:
    %   x = x_center - ye*sin(psi)
    %   y = y_center + ye*cos(psi)
    x_obs = xc - ye_obs * sin(psi);
    y_obs = yc + ye_obs * cos(psi);

    % ---- obstacle rectangle ----
    % half-dimensions
    half_L = obs_L/2;   % forward/back
    half_W = obs_w/2;   % left/right

    % rectangle corners in VEHICLE frame
    corners_local = [ 
        +half_L, +half_W;
        +half_L, -half_W;
        -half_L, -half_W;
        -half_L, +half_W;
    ];

    % rotation matrix
    R = [cos(psi), -sin(psi);
         sin(psi),  cos(psi)];

    % rotate + shift each corner
    corners_global = (R * corners_local')';
    corners_global(:,1) = corners_global(:,1) + x_obs;
    corners_global(:,2) = corners_global(:,2) + y_obs;

    % close the rectangle loop
    corners_global = [corners_global; corners_global(1,:)];

    % ---- plot ----
    hold on;
    fill(corners_global(:,1), corners_global(:,2), 'r', 'FaceAlpha', 0.5, ...
         'EdgeColor', 'k', 'LineWidth', 1.5);
    scatter(x_obs, y_obs, 60, 'k', 'filled'); % obstacle center
end
