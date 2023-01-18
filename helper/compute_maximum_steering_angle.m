function delta_max = compute_maximum_steering_angle(R_c)

    arguments
        R_c (1, 1) double = 0.3
    end

    % parameters
    L = 0.15;
    L_r = 0.075;

    % equation
    syms delta
    eqn = R_c == L / tan(delta) / cos(atan(L_r / L * tan(delta)));
    delta_max = solve(eqn, delta);
    % 27.3117 degrees, 0.4767 radians
end