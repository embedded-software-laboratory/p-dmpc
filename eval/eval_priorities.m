% EVAL_PRIORITIES Study on how the number of level changes with regard to different priority assignments

% undirected coupling graph for scenario
nV = 8;
c = zeros(nV,nV);

% s1-l1-s2-l2-s3-l3-s4-l4
c(1,3) = 1; c(3,1) = 1;
c(1,6) = 1; c(6,1) = 1;
c(1,7) = 1; c(7,1) = 1;
c(1,8) = 1; c(8,1) = 1;
c(2,8) = 1; c(8,2) = 1;
c(2,5) = 1; c(5,2) = 1;
c(2,3) = 1; c(3,2) = 1;
c(2,4) = 1; c(4,2) = 1;
c(3,8) = 1; c(8,3) = 1;
c(3,5) = 1; c(5,3) = 1;
c(4,7) = 1; c(7,4) = 1;
c(4,6) = 1; c(6,4) = 1;
c(5,7) = 1; c(7,5) = 1;
c(6,7) = 1; c(7,6) = 1;
c(6,8) = 1; c(8,6) = 1;

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

fig = figure('position',[100 100 600 630],'color',[1 1 1]);
histogram(x);
ylabel('Number of Planning Levels','Interpreter','LaTex');
title('Number of Planning levels for PBNC-Controllers in Lanelet Scenario','Interpreter','LaTex');