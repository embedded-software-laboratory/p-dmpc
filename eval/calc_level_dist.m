function [ lvl_dist ] = calc_level_dist( c )
%CALC_LEVEL_DIST    Calculate the distribution of number of levels according to different priority assignment.

    nV = size(c, 1);

    % list of all priorities to assign
    prios = 1:nV;

    % all permutations of these priorities (nP = nV!)
    Pm = perms(prios);
    nP = size(Pm,1);

    x = [];
    for ip = 1:nP
        % save directed coupling graph (DAG) in cdag
        cdag = c;
        % current permutation (priority assignment)
        perm = Pm(ip,:);
        % get all pairwise different combinations of vehicles
        for i = 1 : nV-1
            for j = nV : -1 : i+1
                if c(i,j) && c(j,i)
                    % remove edge from lower prio to higher prio
                    if perm(i) < perm(j)
                        cdag(j,i) = 0;
                    else
                        cdag(i,j) = 0;
                    end
                end
            end
        end
        [isDAG, L] = kahn(cdag);
        assert( isDAG, 'Coupling matrix is not a DAG' );
        nLevel = size(L,1);
        x = [x, nLevel];
    end

    lvl_dist = x;
end

