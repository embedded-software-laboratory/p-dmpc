%% Add modules to path
addpath(genpath(pwd));



%% Define scenario
% Obstacles

% Vehicles
% Initial position
% Goal

%% Define motion graph
% Choose Model
model = BicycleModel_constSpeed(2.2,2.2);

% Primitive duration
primitive_dt = 3;

% Trims
nTrims = 5;
velocity = 3;
% trims left to right
u_trims = [0.8, 0.4, 0, -0.4, -0.8];
n_trims = length(u_trims);
t_hard_left = 0;
t_slight_left = 0;
t_straight = 0;
t_slight_right = 0;
t_hard_right = 0;

% Transitions
trim_adjacency = eye(n_trims);
for i = 1:n_trims-1
    trim_adjacency(i,i+1) = 1;
end
% --Mirror to make symmetric
trim_adjacency = trim_adjacency'+triu(trim_adjacency,1);

% Generate graph for one vehicle
motionGraph = MotionGraph(model, u_trims, trim_adjacency, primitive_dt);

% Combine graphs

%% Graph search
% Choose search algorithm

% Search

%% Visualize