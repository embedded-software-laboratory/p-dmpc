%% Add modules to path
% Import tree class
assert(logical(exist('./matlab-tree/', 'dir')));
addpath('./matlab-tree/');
addpath(genpath(pwd));
warning('off','MATLAB:polyshape:repairedBySimplify')

options = selection();

scenario = run_simulation(options);