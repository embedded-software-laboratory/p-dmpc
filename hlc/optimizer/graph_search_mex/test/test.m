clear all;
%addpath(genpath('../..'))
load("test.mat", "iter");
load("test.mat", "scenario");
load("test.mat", "mpa");

optimizer(Function.InitializeWithScenario, scenario, mpa);
%[x0_measured, trim_indices_measured] = optimizer(Function.NaiveCentralizedGraphSearch, iter);
[x0_measured, trim_indices_measured] = optimizer(Function.NaiveMonteCarloCentralizedGraphSearch, iter);
