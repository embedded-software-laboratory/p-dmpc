clear all;
%addpath(genpath('../..'))
load("test.mat", "iter");
load("test.mat", "scenario");

optimizer(Function.InitializeWithScenario, scenario);
%[x0_measured, trim_indices_measured] = optimizer(Function.NaiveCentralizedGraphSearch, iter);
[x0_measured, trim_indices_measured] = optimizer(Function.NaiveMonteCarloCentralizedGraphSearch, iter);

