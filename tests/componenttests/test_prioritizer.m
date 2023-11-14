function tests = test_prioritizer
    tests = functiontests(localfunctions);
end

function TEST_topologically_unique_permutations(testCase)
    % setup
    n_agents = 5;
    rand_stream = RandStream("mt19937ar", "Seed", 0);
    adjacency = rand(rand_stream, n_agents, n_agents) > 0.5;
    adjacency = triu(adjacency, 1) + triu(adjacency, 1)';

    unique_priorities = Prioritizer.unique_priorities(adjacency);
    n_priorities = size(unique_priorities, 2);

    % expected values
    all_priorities = perms(1:n_agents);

    % none are wrong
    testCase.verifyTrue(all(ismember(unique_priorities', all_priorities, "rows")));

    % none are topological duplicates
    topologically_unique_priorities_actual = zeros(0, n_agents);

    for i = 1:n_priorities
        permutation = unique_priorities(:, i)';
        directed_coupling = Prioritizer.directed_coupling_from_priorities(adjacency, permutation);
        coupling_dag = digraph(directed_coupling);
        topological_order = toposort(coupling_dag, Order = 'stable');

        testCase.verifyTrue( ...
            ~ismember(topological_order, topologically_unique_priorities_actual, "rows") ...
        );
        topologically_unique_priorities_actual = [topologically_unique_priorities_actual; topological_order]; %#ok<AGROW>

    end

    % all topologically unique permutations are contained
    % compute topologically unique permutations in a different manner
    topologically_unique_priorities_expected = zeros(0, n_agents);

    for i = 1:size(all_priorities, 1)
        permutation = all_priorities(i, :);
        directed_coupling = Prioritizer.directed_coupling_from_priorities(adjacency, permutation);
        coupling_dag = digraph(directed_coupling);
        topological_order = toposort(coupling_dag, Order = 'stable');

        if ~ismember(topological_order, topologically_unique_priorities_expected, "rows")
            topologically_unique_priorities_expected = [topologically_unique_priorities_expected; topological_order]; %#ok<AGROW>
        end

    end

    testCase.verifyTrue(all(ismember(topologically_unique_priorities_actual, topologically_unique_priorities_expected, "rows")));
    testCase.verifyEqual(size(topologically_unique_priorities_actual, 1), size(topologically_unique_priorities_expected, 1));

end
