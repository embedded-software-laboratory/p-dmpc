classdef CppOptimizer < uint8
    %FUNCTION Enum to give C++ an indication of what to do.

    enumeration
        None(0), ...
        InitializeMex(1), ...
        CentralizedOptimal(2), ...
        CentralizedNaiveMonteCarlo(3), ...
        CentralizedParallelNaiveMonteCarlo(4), ...
        CentralizedNaiveMonteCarloPolymorphic(5), ...
        CentralizedNaiveMonteCarloPolymorphicParallel(6), ...
        CentralizedOptimalMemorySaving(7), ...
        CentralizedOptimalPolymorphic(8), ...
        CentralizedOptimalIncremental(9), ...
        CentralizedOptimalAStarParallelization(10), ...
        CentralizedOptimalNodeParallelization(11), ...
        CentralizedMonteCarloTreeSearchBasic(12), ...
        CentralizedMonteCarloTreeSearch(13), ...
        CentralizedNaiveMonteCarloLimitingSearchRange(14), ...
        CentralizedNaiveMonteCarloLimitingSearchRangeParallel(15), ...
        CentralizedConflictBased(16), ...
        CentralizedCreatingDistribution(18), ...
        CentralizedGrouping(19), ...
        CentralizedConflictBasedFast(20), ...
        GraphSearchPBOptimal(21), ...
        GraphSearchPBIncrementalOptimal(22)
    end
end