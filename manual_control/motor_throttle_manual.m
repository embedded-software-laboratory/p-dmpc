function result = motor_throttle_manual(...
    accelerator_pedal_position, brake_pedal_position, speed...
)
arguments
    accelerator_pedal_position (1,1) double = -1;
    brake_pedal_position (1,1) double = -1;
    speed (1,1) double = 0;
end

acceleration_desired = compute_desired_acceleration( ...
    accelerator_pedal_position, brake_pedal_position, speed ...
);

result = compute_motor_throttle(acceleration_desired, speed);

end

function result = compute_desired_acceleration(accelerator_pedal_position, brake_pedal_position, speed)
    acceleration_from_brake = motor_throttle_from_pedal_and_max_acceleration( ...
        brake_pedal_position, compute_min_acceleration() ...
    );

    acceleration_from_accelerator = motor_throttle_from_pedal_and_max_acceleration( ...
        accelerator_pedal_position, compute_max_acceleration(speed) ...
    );
    result = acceleration_from_brake + acceleration_from_accelerator;
end

function result = compute_min_acceleration()
    % see https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/-/blob/master/vehicleModels_commonRoad.pdf
    scale = 1/18;
    max_acceleration = 11.5 * scale;
    result = -max_acceleration;
end

function result = compute_max_acceleration(speed)
    % see https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/-/blob/master/vehicleModels_commonRoad.pdf
    scale = 1/18;
    max_acceleration = 11.5 * scale;
    speed_no_slip = 5 * scale;
    
    if (speed < speed_no_slip)
        result = max_acceleration;
    else
        result = max_acceleration * speed_no_slip ./ speed;
    end
end

function result = motor_throttle_from_pedal_and_max_acceleration( ...
    pedal_position, max_acceleration ...
)
    min_acceleration = 0;
    d_acceleration = max_acceleration - min_acceleration;
    min_pedal_position = -1;
    max_pedal_position = 1;
    d_pedal = max_pedal_position - min_pedal_position;
    result = (pedal_position - min_pedal_position) / d_pedal * d_acceleration ...
        + min_acceleration;
end

function result = compute_motor_throttle(acceleration_desired, speed)
    % https://www.sciencedirect.com/science/article/pii/S2405896320324319
    % TODO no backwards motion from braking
    % TODO lose speed faster
    p5 = -1.42;
    p6 =  6.90;
    p7 =  1.34;
    x = ( acceleration_desired - p5 * speed ) / p6;
    result = sign(x) * nthroot( abs(x) , p7 );
end