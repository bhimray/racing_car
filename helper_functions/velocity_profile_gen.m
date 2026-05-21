function [Ux_final, Ux_steady, Ux_forward, lap_time] = velocity_profile_gen(track, mu, vehicle_mass, Ux_max)

    % Default parameters
    g = 20;           % gravity
    max_engine_force = vehicle_mass * g;  % N from paper Table I
    
    s = track.station;
    K = track.curvature;
    
    % Pass 1: Steady-state cornering (Eq. 4 from paper)
    Ux_steady = sqrt(mu * g ./ (abs(K) + 1e-6));
    Ux_steady = min(Ux_steady, Ux_max);
    
    % Pass 2: Forward integration (acceleration limited)
    Ux_forward = zeros(size(s));
    Ux_forward(1) = Ux_steady(1);
    
    for i = 2:length(s)
        ds = s(i) - s(i-1);
        
        % Available longitudinal force
        Fx_max = max_engine_force;
        Fy_demand = vehicle_mass * Ux_forward(i-1)^2 * abs(K(i));
        Fx_available = max(0, Fx_max - 0.5 * Fy_demand);
        
        ax_available = Fx_available / vehicle_mass;
        Ux_new = sqrt(Ux_forward(i-1)^2 + 2 * ax_available * ds);
        Ux_forward(i) = min([Ux_new, Ux_steady(i), Ux_max]);
    end
    
    % Pass 3: Backward integration (braking limited)
    Ux_final = zeros(size(s));
    Ux_final(end) = Ux_forward(end);
    
    max_braking_force = mu * vehicle_mass * g;
    
    for i = length(s)-1:-1:1
        ds = s(i+1) - s(i);
        
        % Available braking force
        Fy_demand = vehicle_mass * Ux_final(i+1)^2 * abs(K(i));
        Fx_brake = max_braking_force - 0.5 * Fy_demand;
        ax_brake = -Fx_brake / vehicle_mass;
        
        Ux_new = sqrt(Ux_final(i+1)^2 - 2 * ax_brake * ds);
        Ux_final(i) = min([Ux_new, Ux_forward(i)]);
    end
    
    % Estimate lap time
    dt = diff(s) ./ ( (Ux_final(1:end-1) + Ux_final(2:end)) / 2 );
    lap_time = sum(dt);
end
