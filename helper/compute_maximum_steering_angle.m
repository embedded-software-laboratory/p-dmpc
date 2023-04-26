function delta_max = compute_maximum_steering_angle(R_c)

    arguments
        R_c (1, 1) double = 0.35 % Measured with steering_servo=-1
    end

    % parameters
    L = 0.15;
    L_r = 0.075;

    % equation
    syms delta
    eqn = R_c == L / tan(delta) / cos(atan(L_r / L * tan(delta)));
    delta_max = solve(eqn, delta);
    % from R_c = 0.35 m
    % delta_max = 23.6901 degrees / 0.4135 radians
end
