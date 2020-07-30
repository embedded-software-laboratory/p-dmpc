function search_path = return_path(search_graph)
%RETURN_PATH returns the path to the closest node

    search_path = [];

    % Find the node closest to the target
    for i = 1:height(search_graph.Nodes)

        min_value = inf;
        min_id = 0;

        if search_graph.Nodes{i, 7} < min_value
            min_value = search_graph.Nodes{i, 7};
            min_id = i;
        end
    end

    path = shortestpath(search_graph,1,min_id);
    disp(path);
    disp(length(path));

    for i = 1:length(path)

        x = search_graph.Nodes{path(i), 4};
        y = search_graph.Nodes{path(i), 5};
        yaw = search_graph.Nodes{path(i), 6};

        % If there is a succesor in path add its trim else default to 0
        if (i + 1) <= length(path)
            next_trim = search_graph.Nodes{path(i + 1), 3};
        else
            next_trim = 0;
        end

        search_path = [search_path; x y yaw next_trim];

    end

end

