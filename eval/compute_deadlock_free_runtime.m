function [nSteps, t] = compute_deadlock_free_runtime(result)
% COMPUTE_DEADLOCK_FREE_RUNTIME computes the deadlock-free runtime of a result
% struct

    % TODO detect deadlock from standstill
    % option 1: "heuristic" -- stopping for certain time
    % option 2: check all permutations of priorities for involved
    % vehicles. If no movement, then deadlock
    nSteps = find(result.is_deadlock,1,'first');
    if isempty(nSteps)
        t = result.t_total;
        nSteps = result.nSteps;
    else
        t = nSteps * result.scenario.options.dt;
    end
end