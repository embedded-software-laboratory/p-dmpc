function runtimes = calculate_subcontroller_runtime_pb(experiment_result)
    % CALCULATE_SUBCONTROLLER_RUNTIME_PB    Calculate the subcontroller runtimes for a priority-base controller based on the "longest"
    %                                       path in the coupling graph (longest means highest cumulative runtime)

    arguments
        experiment_result (1, 1) ExperimentResult;
    end

    %TODO calculate subcontroller_runtime from other timings
    [nVeh, n_steps] = size(experiment_result.subcontroller_runtime);
    runtimes = zeros(1, n_steps);

    % build DAG from levels and coupling adjacency
    for i = 1:n_steps
        runtime = experiment_result.subcontroller_runtime(:, i);
        % --- modified only for eval/prio-paper --
        groups = experiment_result.coupling{1}.groups;
        adjacency = experiment_result.coupling{1}.adjacency;
        visited = [];

        for j = 1:length(groups)
            group = groups(j);
            members = group.members;

            for m = members

                for v = visited
                    % remove edges connecting a vertex with an upper level
                    if adjacency(m, v)
                        adjacency(m, v) = 0;
                    end

                end

            end

            visited = [visited, members];
        end

        leaves = find(sum(adjacency') == 0);

        % add source and target vertex
        adjacency(:, end + 1) = 0;
        adjacency(end + 1, :) = 0;
        s = size(adjacency, 1);
        adjacency(:, end + 1) = 0;
        adjacency(end + 1, :) = 0;
        t = size(adjacency, 1);

        % add edges from source to first level
        adjacency(s, groups(1).members) = 1;
        % add edges from leaves to target
        adjacency(leaves, t) = 1;

        % add runtimes as weights of incoming edges
        neg_runtime = runtime * -1;

        for j = 1:nVeh
            adjacency(find(adjacency(:, j)), j) = neg_runtime(j);
        end

        % construct digraph
        g = digraph(adjacency);
        p = shortestpath(g, s, t);
        % remove source and target
        p(1) = [];
        p(end) = [];

        % add runtimes for overall runtime of timestep
        runtimes(i) = sum(runtime(p));

    end

end
