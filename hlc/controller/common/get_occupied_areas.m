function occupied_areas = get_occupied_areas(x, y, yaw, length, width, offset)
    % the function calculates the local occupied area based on the
    % dimensions of the vehicle and translates it to the global frame
    %
    % Output:
    %   occupied_areas (1, 1) struct with the fields
    %       normal_offset (2, :) double [x; y]
    %       without_offset (2, :) double [x; y]

    arguments
        x (1, 1) double % current x coordinate
        y (1, 1) double % current y coordinate
        yaw (1, 1) double % current yaw
        length (1, 1) double % length of the vehicle
        width (1, 1) double % width of the vehicle
        offset (1, 1) double % offset for the occupied area
    end

    % get vehicles currently occupied area with offset
    % repeat the first entry to enclose the shape
    x_rec1 = [-1, -1, 1, 1, -1] * (length / 2 + offset);
    y_rec1 = [-1, 1, 1, -1, -1] * (width / 2 + offset);
    [x_rec2, y_rec2] = translate_global(yaw, x, y, x_rec1, y_rec1);
    occupied_areas.normal_offset = [x_rec2; y_rec2];

    % get vehicles currently occupied area without offset
    % repeat the first entry to enclose the shape
    x_rec1_without_offset = [-1, -1, 1, 1, -1] * (length / 2);
    y_rec1_without_offset = [-1, 1, 1, -1, -1] * (width / 2);
    [x_rec2_without_offset, y_rec2_without_offset] = translate_global(yaw, x, y, x_rec1_without_offset, y_rec1_without_offset);
    occupied_areas.without_offset = [x_rec2_without_offset; y_rec2_without_offset];

end
