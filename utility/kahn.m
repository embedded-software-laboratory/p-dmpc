function L = kahn(A)
    % Given the directed coupling matrix,
    % this function returns the computation level of each vehicle.
    current_level = 1;
    L = zeros(1, length(A));
    in_D = sum(A, 1);

    sorted_vertices = false(1, length(A));

    while ~all(sorted_vertices)
        source_vertices = (in_D == 0);

        L(source_vertices) = current_level;
        % remove outgoing edges of sources
        A(source_vertices, :) = 0;

        sorted_vertices(source_vertices) = 1;
        in_D = sum(A, 1);
        in_D(sorted_vertices) = 1;

        current_level = current_level + 1;
    end

end
