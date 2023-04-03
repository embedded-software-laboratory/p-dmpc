function [cmap, n_colors_max] = discrete_colormap()
    % DISCRETE_COLORMAP This function initializes a discrete colormap based on RWTH colors.
    %
    % OUTPUT:
    %   cmap: the discrete colormap of size (n_colors_max, 3)
    %   n_colors_max: the number of different colors

    %% Definitions.
    % Define some base colors from the RWTH palette.
    colors = [
              246 168 0; % RWTH orange
              204 7 30; % RWTH red
              161 16 53; % RWTH bordeaux
              97 33 88; % RWTH violet
              122 111 172; % RWTH purple
              64 127 183; % RWTH blue 75 %
              0 84 159 % RWTH blue
              ] / 255;
    % Define where between the base colors interpolated colors are inserted:
    % The number at index i tells you that between colors i and i+1 splits(i) colors are interpolated and inserted into the colormap.
    splits = [4, 1, 1, 5, 0, 2];
    cmap = divide_colorspace(colors, splits);

    %% Define outputs.
    % Ensure that all values are at most 1 as rounding errors could lead to values slightly larger that 1.
    cmap = min(cmap, 1);
    % Also return the number of colors in the computed discrete colormap.
    n_colors_max = size(cmap, 1);
end

% Helper function to interpolate colors and insert the interpolated colors into a colormap.
function [cmap] = divide_colorspace(colors, splits)

    if isempty(colors)
        % If no base colors are given, the resulting colormap is empty.
        cmap = [];
    else
        % The size of the resulting colormap is given by the number of base colors plus interpolated colors.
        cmap = zeros(length(colors) + sum(splits), 3);
        assert(size(colors, 1) == length(splits) + 1);
        % Start with the first base color and then iteratively add interpolated colors defined in splits and then the next base color.
        cmap(1, :) = colors(1, :);
        i_row = 2;

        for i_color = 2:size(colors, 1)
            n_splits = splits(i_color - 1);

            for i_split = 1:n_splits
                % Use linear interpolation.
                cmap(i_row, :) = ((n_splits + 1 - i_split) * colors(i_color - 1, :) + i_split * colors(i_color, :)) / (n_splits + 1);
                i_row = i_row + 1;
            end

            cmap(i_row, :) = colors(i_color, :);
            i_row = i_row + 1;
        end

    end

end
