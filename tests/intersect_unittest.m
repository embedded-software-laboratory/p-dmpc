function tests = intersect_unittest
% INTERSECT_SAT_UNITTEST    Test the INTERSECT_SAT function for compability with
%                           polygon-lanelet & polygon-polygon intersection test.
    
    tests = functiontests(localfunctions);
end

function testLaneletPos(testcase)
    [lanelets, ~] = intersection_lanelets;
    lanelet_idx = [3,18,6];
    collision = false;
    shape = [0, 5, 5, 0; 0, 0, -5, -5];
    for i = lanelet_idx
        lanelet = lanelets{i};
        if intersect_lanelets(shape,lanelet)
            collision = true;
            break;
        end
    end
    actSolution = collision;
    expSolution = true;
    verifyEqual(testcase,actSolution,expSolution)
end

function testLaneletNegLeft(testcase)
    [lanelets, ~] = intersection_lanelets;
    lanelet_idx = [5,19,8];
    collision = false;
    shape = [-1.5, -1, -2.5, -2.5; 1, -1, -1, 0];
    for i = lanelet_idx
        lanelet = lanelets{i};
        if intersect_lanelets(shape,lanelet)
            collision = true;
            break;
        end
    end
    actSolution = collision;
    expSolution = false;
    verifyEqual(testcase,actSolution,expSolution)
end

function testLaneletNegRight(testcase)
    [lanelets, ~] = intersection_lanelets;
    lanelet_idx = [5,15,5];
    collision = false;
    shape = [2.1, 3, 3.5, 2.5; -2.1, -2.1, -3.5, -5];
    for i = lanelet_idx
        lanelet = lanelets{i};
        if intersect_lanelets(shape,lanelet)
            collision = true;
            break;
        end
    end
    actSolution = collision;
    expSolution = false;
    verifyEqual(testcase,actSolution,expSolution)
end

function testPolygonPos(testcase)
    shape1 = [  -7.0749   -2.8728    9.8889   21.3024   15.3469    7.9387
                -6.4707  -12.1152  -24.4428   -3.0950   19.3838   18.7030   ];
    shape2 = shape1 - [5;5];
    actSolution = intersect_sat(shape1,shape2);
    expSolution = true;
    verifyEqual(testcase,actSolution,expSolution)
end

function testPolygonNeg(testcase)
    shape1 = [  -7.0749   -2.8728    9.8889   21.3024   15.3469    7.9387
                -6.4707  -12.1152  -24.4428   -3.0950   19.3838   18.7030   ];
    shape2 = shape1 - [40;40];
    actSolution = intersect_sat(shape1,shape2);
    expSolution = false;
    verifyEqual(testcase,actSolution,expSolution)
end
