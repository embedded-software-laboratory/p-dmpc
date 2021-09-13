% EVAL_PRIORITIES Study on how the number of level changes with regard to different priority assignments


%% construct scenarios

cs = {};

% undirected coupling graph for scenario
nV = 8;

% s1-l1-s2-l2-s3-l3-s4-l4 (scenario #1)
c = zeros(nV,nV);
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
c(4,5) = 1; c(5,4) = 1;
c(5,7) = 1; c(7,5) = 1;
c(6,7) = 1; c(7,6) = 1;
c(6,8) = 1; c(8,6) = 1;

cs{end+1}= c;

% r1-l1-r2-l2-r3-l3-r4-l4 (scenario #2)
c = zeros(nV,nV);
c(2,4) = 1; c(4,2) = 1;
c(4,6) = 1; c(6,4) = 1;
c(6,8) = 1; c(8,6) = 1;
c(8,2) = 1; c(2,8) = 1;

cs{end+1}= c;

% s1-l1-r2-l2-s3-l3-r4-l4 (scenario #3)
c = zeros(nV,nV);
c(2,4) = 1; c(4,2) = 1;
c(4,6) = 1; c(6,4) = 1;
c(6,8) = 1; c(8,6) = 1;
c(8,2) = 1; c(2,8) = 1;
c(5,2) = 1; c(2,5) = 1;
c(5,3) = 1; c(3,5) = 1;
c(5,4) = 1; c(4,5) = 1;
c(1,8) = 1; c(8,1) = 1;
c(1,7) = 1; c(7,1) = 1;
c(1,6) = 1; c(6,1) = 1;

cs{end+1}= c;

% s1-l1-r2-l2-r3-l3-r4-l4 (scenario #4)
c = zeros(nV,nV);
c(2,4) = 1; c(4,2) = 1;
c(4,6) = 1; c(6,4) = 1;
c(6,8) = 1; c(8,6) = 1;
c(8,2) = 1; c(2,8) = 1;
c(1,8) = 1; c(8,1) = 1;
c(1,7) = 1; c(7,1) = 1;
c(1,6) = 1; c(6,1) = 1;

cs{end+1}= c;

% r1-l1-s2-l2-s3-l3-s4-l4 (scenario #5)
c = zeros(nV,nV);
c(1,3) = 1; c(3,1) = 1;
c(2,8) = 1; c(8,2) = 1;
c(2,5) = 1; c(5,2) = 1;
c(2,3) = 1; c(3,2) = 1;
c(2,4) = 1; c(4,2) = 1;
c(3,8) = 1; c(8,3) = 1;
c(3,5) = 1; c(5,3) = 1;
c(4,7) = 1; c(7,4) = 1;
c(4,6) = 1; c(6,4) = 1;
c(4,5) = 1; c(5,4) = 1;
c(5,7) = 1; c(7,5) = 1;
c(6,7) = 1; c(7,6) = 1;
c(6,8) = 1; c(8,6) = 1;

cs{end+1}= c;

% r1-l1-r2-l2-s3-l3-s4-l4 (scenario #6)
c = zeros(nV,nV);
c(2,8) = 1; c(8,2) = 1;
c(2,5) = 1; c(5,2) = 1;
c(2,4) = 1; c(4,2) = 1;
c(3,5) = 1; c(5,3) = 1;
c(4,7) = 1; c(7,4) = 1;
c(4,6) = 1; c(6,4) = 1;
c(4,5) = 1; c(5,4) = 1;
c(5,7) = 1; c(7,5) = 1;
c(6,7) = 1; c(7,6) = 1;
c(6,8) = 1; c(8,6) = 1;

cs{end+1}= c;


for i = 1:length(cs)
    lvl_dist = calc_level_dist(cs{i});

    fig = figure('position',[100 100 600 630],'color',[1 1 1]);
    histogram(lvl_dist);
    ylabel('Number of Planning Levels','Interpreter','LaTex');
    title(sprintf('Number of Planning levels for PBNC-Controllers in Lanelet Scenario %i',i),'Interpreter','latex'); 
end