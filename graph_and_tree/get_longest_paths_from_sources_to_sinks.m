function path_info = get_longest_paths_from_sources_to_sinks(M)
% GET_LONGEST_PATHS_FROM_SOURCES_TO_SINKS Returns a struct contains the
% all longest paths starting from source vertices to sink vertices.

    directed_adjacency = (M ~= 0);
    source_vertices = find(all(directed_adjacency==0, 1)); % get all source vertices of the DAG
    sink_vertices = find(all(directed_adjacency==0, 2))'; % get all sink vertices of the DAG
    n_sources = length(source_vertices);
    n_sinks = length(sink_vertices);

    path_info(n_sources*n_sinks) = struct('source',[],'sink',[],'path',[],'length',[]);

    count = 1;

    % Get the distances between source vertices and sink vertices. 
    % NOTE that here purposely convert the edge-weights to be negative to
    % get the longest distance via MATLAB function `shortestpath`.
    G_negative_weighted = digraph(-directed_adjacency);
    for i = 1:n_sources
        source_i = source_vertices(i);
        for j = 1:n_sinks
            sink_j = sink_vertices(j);
            path_info(count).source = source_i;
            path_info(count).sink = sink_j;
            path_info(count).path = shortestpath(G_negative_weighted,source_i,sink_j);
            path_info(count).length = length(path_info(count).path);
            count = count + 1;
        end
    end

    % order the paths according to their lengths
    [~,descending_order] = sort([path_info.length],'descend');
    path_info = path_info(descending_order);

end