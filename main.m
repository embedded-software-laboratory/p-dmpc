%% Add modules to path
addpath(genpath(pwd));



%% Define scenario
% Obstacles

% Vehicles
% Initial position
% Goal

%% Define motion graph
% Choose Model
model = BicycleModel(2.2,2.2);

% Primitive duration
primitive_dt = 1;

% Trims
nTrims = 5; % useless (?)
velocity = 3;

% trims left to right
load('trim_inputs');
n_trims = length(u_trims);

% Transitions (maneuver)
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
