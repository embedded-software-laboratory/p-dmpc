function [lanelets, collision] = intersection_lanelets()
% INTERSECTION_LANELETS     Constructor for lanelets of one-lane intersection

    lanelets = cell(1,0);
    
    %% incoming/outgoing
    % north incoming - 1
    l = zeros(2,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [-5, -5];
    l(:,LaneletInfo.ry) = [20, 5];
    l(:,LaneletInfo.lx) = [0, 0];
    l(:,LaneletInfo.ly) = [20, 5];
    l(:,LaneletInfo.cx) = [-2.5, -2.5];
    l(:,LaneletInfo.cy) = [20, 5];
    lanelets{end+1} = l;
    
    % north outgoing - 2
    l = zeros(2,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [5, 5];
    l(:,LaneletInfo.ry) = [5, 20];
    l(:,LaneletInfo.lx) = [0, 0];
    l(:,LaneletInfo.ly) = [5, 20];
    l(:,LaneletInfo.cx) = [2.5, 2.5];
    l(:,LaneletInfo.cy) = [5, 20];
    lanelets{end+1} = l;
    
    % east incoming - 3
    l = zeros(2,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [20, 5];
    l(:,LaneletInfo.ry) = [5, 5];
    l(:,LaneletInfo.lx) = [20, 5];
    l(:,LaneletInfo.ly) = [0, 0];
    l(:,LaneletInfo.cx) = [20, 5];
    l(:,LaneletInfo.cy) = [2.5, 2.5];
    lanelets{end+1} = l;
    
    % east outgoing - 4
    l = zeros(2,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [5, 20];
    l(:,LaneletInfo.ry) = [-5, -5];
    l(:,LaneletInfo.lx) = [5, 20];
    l(:,LaneletInfo.ly) = [0, 0];
    l(:,LaneletInfo.cx) = [5, 20];
    l(:,LaneletInfo.cy) = [-2.5, -2.5];
    lanelets{end+1} = l;
    
    % south incoming - 5
    l = zeros(2,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [5, 5];
    l(:,LaneletInfo.ry) = [-20, -5];
    l(:,LaneletInfo.lx) = [0, 0];
    l(:,LaneletInfo.ly) = [-20, -5];
    l(:,LaneletInfo.cx) = [2.5, 2.5];
    l(:,LaneletInfo.cy) = [-20, -5];
    lanelets{end+1} = l;
    
    % south outgoing - 6
    l = zeros(2,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [-5, -5];
    l(:,LaneletInfo.ry) = [-5, -20];
    l(:,LaneletInfo.lx) = [0, 0];
    l(:,LaneletInfo.ly) = [-5, -20];
    l(:,LaneletInfo.cx) = [-2.5, -2.5];
    l(:,LaneletInfo.cy) = [-5, -20];
    lanelets{end+1} = l;
    
    % west incoming - 7
    l = zeros(2,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [-20, -5];
    l(:,LaneletInfo.ry) = [-5, -5];
    l(:,LaneletInfo.lx) = [-20, -5];
    l(:,LaneletInfo.ly) = [0, 0];
    l(:,LaneletInfo.cx) = [-20, -5];
    l(:,LaneletInfo.cy) = [-2.5, -2.5];
    lanelets{end+1} = l;
    
    % west outgoing - 8
    l = zeros(2,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [-5, -20];
    l(:,LaneletInfo.ry) = [5, 5];
    l(:,LaneletInfo.lx) = [-5, -20];
    l(:,LaneletInfo.ly) = [0, 0];
    l(:,LaneletInfo.cx) = [-5, -20];
    l(:,LaneletInfo.cy) = [2.5, 2.5];
    lanelets{end+1} = l;
    
    
    %% straight connections
    %connecting north-south - 9
    l = zeros(2,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [-5, -5];
    l(:,LaneletInfo.ry) = [5, -5];
    l(:,LaneletInfo.lx) = [0, 0];
    l(:,LaneletInfo.ly) = [5, -5];
    l(:,LaneletInfo.cx) = [-2.5, -2.5];
    l(:,LaneletInfo.cy) = [5, -5];
    lanelets{end+1} = l;
    
    % connecting east-west - 10
    l = zeros(2,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [5, -5];
    l(:,LaneletInfo.ry) = [5, 5];
    l(:,LaneletInfo.lx) = [5, -5];
    l(:,LaneletInfo.ly) = [0, 0];
    l(:,LaneletInfo.cx) = [5, -5];
    l(:,LaneletInfo.cy) = [2.5, 2.5];
    lanelets{end+1} = l;
    
    %connecting south-north - 11
    l = zeros(2,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [5, 5];
    l(:,LaneletInfo.ry) = [-5, 5];
    l(:,LaneletInfo.lx) = [0, 0];
    l(:,LaneletInfo.ly) = [-5, 5];
    l(:,LaneletInfo.cx) = [2.5, 2.5];
    l(:,LaneletInfo.cy) = [-5, 5];
    lanelets{end+1} = l;
    
    % connecting west-east - 12
    l = zeros(2,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [-5, 5];
    l(:,LaneletInfo.ry) = [-5, -5];
    l(:,LaneletInfo.lx) = [-5, 5];
    l(:,LaneletInfo.ly) = [0, 0];
    l(:,LaneletInfo.cx) = [-5, 5];
    l(:,LaneletInfo.cy) = [-2.5, -2.5];
    lanelets{end+1} = l;
    
    %% right connections
    % connecting north-west - 13
    l = zeros(3,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [-5, -5, -5];
    l(:,LaneletInfo.ry) = [5, 5, 5];
    l(:,LaneletInfo.lx) = [0, -2, -5];
    l(:,LaneletInfo.ly) = [5, 2, 0];
    l(:,LaneletInfo.cx) = [-2.5, -3.5, -5];
    l(:,LaneletInfo.cy) = [5, 3.5, 2.5];
    lanelets{end+1} = l;
    
    % connecting east-north - 14
    l = zeros(3,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [5, 5, 5];
    l(:,LaneletInfo.ry) = [5, 5, 5];
    l(:,LaneletInfo.lx) = [5, 2, 0];
    l(:,LaneletInfo.ly) = [0, 2, 5];
    l(:,LaneletInfo.cx) = [5, 3.5, 2.5];
    l(:,LaneletInfo.cy) = [2.5, 3.5, 5];
    lanelets{end+1} = l;
    
    % connecting south-east - 15
    l = zeros(3,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [5, 5, 5];
    l(:,LaneletInfo.ry) = [-5, -5, -5];
    l(:,LaneletInfo.lx) = [0, 2, 5];
    l(:,LaneletInfo.ly) = [-5, -2, 0];
    l(:,LaneletInfo.cx) = [2.5, 3.5, 5];
    l(:,LaneletInfo.cy) = [-5, -3.5, -2.5];
    lanelets{end+1} = l;
    
    % connecting west-south - 16
    l = zeros(3,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [-5, -5, -5];
    l(:,LaneletInfo.ry) = [-5, -5, -5];
    l(:,LaneletInfo.lx) = [-5, -2, 0];
    l(:,LaneletInfo.ly) = [0, -2, -5];
    l(:,LaneletInfo.cx) = [-5, -3.5, -2.5];
    l(:,LaneletInfo.cy) = [-2.5, -3.5, -5];
    lanelets{end+1} = l;
    
    %% left connections
    % connecting north-east - 17
    l = zeros(3,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [-5, 0, 5];
    l(:,LaneletInfo.ry) = [5, 0, -5];
    l(:,LaneletInfo.lx) = [0, 2, 5];
    l(:,LaneletInfo.ly) = [5, 2, 0];
    l(:,LaneletInfo.cx) = [-2.5, 1, 5];
    l(:,LaneletInfo.cy) = [5, 1, -2.5];
    lanelets{end+1} = l;
    
    % connecting east-south - 18
    l = zeros(3,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [5, 0, -5];
    l(:,LaneletInfo.ry) = [5, 0, -5];
    l(:,LaneletInfo.lx) = [5, 2, 0];
    l(:,LaneletInfo.ly) = [0, -2, -5];
    l(:,LaneletInfo.cx) = [5, 1, -2.5];
    l(:,LaneletInfo.cy) = [2.5, -1, -5];
    lanelets{end+1} = l;
    
    % connecting south-west - 19
    l = zeros(3,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [5, 0, -5];
    l(:,LaneletInfo.ry) = [-5, 0, 5];
    l(:,LaneletInfo.lx) = [0, -2, -5];
    l(:,LaneletInfo.ly) = [-5, -2, 0];
    l(:,LaneletInfo.cx) = [2.5, -1, -5];
    l(:,LaneletInfo.cy) = [-5, -1, 2.5];
    lanelets{end+1} = l;
    
    % connecting west-north - 20
    l = zeros(3,LaneletInfo.nCols);
    l(:,LaneletInfo.rx) = [-5, 0, 5];
    l(:,LaneletInfo.ry) = [-5, 0, 5];
    l(:,LaneletInfo.lx) = [-5, -2, 0];
    l(:,LaneletInfo.ly) = [0, 2, 5];
    l(:,LaneletInfo.cx) = [-5, -1, 2.5];
    l(:,LaneletInfo.cy) = [-2.5, 1, 5];
    lanelets{end+1} = l;
    
    %% collision pair matrix
    % number of lanelet segments
    nLanelets = length(lanelets);
    
    c = zeros(nLanelets, nLanelets);
    
    % straight north-south
    c(9,10) = 1;
    c(9,12) = 1;
    c(9,13) = 1;
    c(9,16) = 1;
    c(9,17) = 1;
    c(9,18) = 1;
    c(9,19) = 1;
    c(9,20) = 1;
    
    % straight east-west
    c(10,11) = 1;
    c(10,13) = 1;
    c(10,14) = 1;
    c(10,17) = 1;
    c(10,18) = 1;
    c(10,19) = 1;
    c(10,20) = 1;
    
    % straight south-north
    c(11,12) = 1;
    c(11,14) = 1;
    c(11,15) = 1;
    c(11,17) = 1;
    c(11,18) = 1;
    c(11,19) = 1;
    c(11,20) = 1;
    
    % straight west-east
    c(12,15) = 1;
    c(12,16) = 1;
    c(12,17) = 1;
    c(12,18) = 1;
    c(12,19) = 1;
    c(12,20) = 1;
    
    % right north-west
    c(13,17) = 1;
    c(13,19) = 1;
    
    % right east-north
    c(14,18) = 1;
    c(14,20) = 1;
    
    % right south-east
    c(15,17) = 1;
    c(15,19) = 1;
    
    % right west-south
    c(16,18) = 1;
    c(16,20) = 1;
    
    % left north-east
    c(17,18) = 1;
    c(17,20) = 1;
    
    % left east-south
    c(18,19) = 1;
    
    % left south-east
    c(19,20) = 1;
    
    
    
    collision = c + c' + eye(nLanelets, nLanelets); % same lanelet is everytime a collision
    
end