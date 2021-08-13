function tests = coupling_lanelets_unittest
% COUPLING_LANELETS_UNITTEST    Test the undirected coupling adjacency based on lanelet indices
    
    tests = functiontests(localfunctions);
end

function TEST_coupling_adjacency_ll_WITH_4_veh_SHOULD_detect_3_couplings(testcase)
    [~, collision] = intersection_lanelets;
    lanelet_idx = [3,14,2; 5,11,2; 1,9,6; 5,15,4];
    coupling_adjacency = [0,1,0,0; 1,0,0,1; 0,0,0,0; 0,1,0,0];
    
    actSolution = coupling_adjacency_lanelets(lanelet_idx,collision);
    expSolution = coupling_adjacency;
    verifyEqual(testcase,actSolution,expSolution)
end
