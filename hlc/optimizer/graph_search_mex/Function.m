classdef Function < uint8
    %FUNCTION Enum to give C++ an indication of what to do.

    enumeration
        CheckMexFunction(0), ...
        InitializeWithScenario(1), ...
        GraphSearchCentralizedOptimal(2), ...
        GraphSearchCentralizedNaiveMonteCarlo(3), ...
        GraphSearchCentralizedParallelNaiveMonteCarlo(4), ...
        GraphSearchCentralizedNaiveMonteCarloPolymorphic(5), ...
        GraphSearchCentralizedParallelNaiveMonteCarloPolymorphic(6), ...
    	GraphSearchCentralizedOptimalMemorySaving(7), ...
        GraphSearchCentralizedOptimalPolymorphic(8), ...
    	GraphSearchCentralizedOptimalPolymorphicSpeedHeuristic(9)
    end
end