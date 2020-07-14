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

% Generate graph for two vehicles
motionGraph1 = MotionGraph(model, u_trims, trim_adjacency, primitive_dt);
motionGraph2 = MotionGraph(model, u_trims, trim_adjacency, primitive_dt);
motionGraph3 = MotionGraph(model, u_trims(1:8,:), trim_adjacency, primitive_dt);

% make motionGraph Tupel
motionGraphList = [motionGraph1 , motionGraph2 , motionGraph3];

% Combine graphs
combinedGraph = CombinedGraph(motionGraphList);
%% Graph search
% Choose search algorithm

% Search

%% Visualize
