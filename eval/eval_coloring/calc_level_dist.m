function lvl_dist = calc_level_dist(c)
    %CALC_LEVEL_DIST    Calculate the distribution of number of levels according to different priority assignment.

    nV = size(c, 1);

    % unique permutations of these priorities
    Pm = Prioritizer.unique_priorities(c)';
    nP = size(Pm, 1);

    lvl_dist = zeros(1, nP);

    for ip = 1:nP
        % save directed coupling graph (DAG) in cdag
        cdag = c;
        % current permutation (priority assignment)
        perm = Pm(ip, :);
        % get all pairwise different combinations of vehicles
        for i = 1:nV - 1

            for j = nV:-1:i + 1

                if c(i, j) && c(j, i)
                    % remove edge from lower prio to higher prio
                    if perm(i) < perm(j)
                        cdag(j, i) = 0;
                    else
                        cdag(i, j) = 0;
                    end

                end

            end

        end

        levels = kahn(cdag);
        lvl_dist(ip) = max(levels);

    end

end
