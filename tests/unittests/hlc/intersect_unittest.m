function tests = intersect_unittest
    % INTERSECT_SAT_UNITTEST    Test the INTERSECT_SAT function for compability with
    %                           polygon-lanelet & polygon-polygon intersection test.

    tests = functiontests(localfunctions);
end

function TEST_WITH_lanelet_inside_of_shape_SHOULD_return_true(testcase)
    road_data = RoadDataCommonRoad().get_road_data();
    lanelet = road_data.lanelets{1};
    shape = [0, 5, 5, 0; 0, 0, 5, 5];

    actSolution = intersect_lanelets(shape, lanelet);
    expSolution = true;
    verifyEqual(testcase, actSolution, expSolution)
end

function TEST_WITH_shape_inside_of_lanelet_SHOULD_return_false(testcase)
    road_data = RoadDataCommonRoad().get_road_data();
    lanelet = road_data.lanelets{1};
    shape = [2.4, 2.5, 2.5, 2.4; 3.7, 3.7, 3.8, 3.8];

    actSolution = intersect_lanelets(shape, lanelet);
    expSolution = false;
    verifyEqual(testcase, actSolution, expSolution)
end

function TEST_WITH_lanelet_left_boundary_SHOULD_return_true(testcase)
    road_data = RoadDataCommonRoad().get_road_data();
    lanelet = road_data.lanelets{1};
    shape = [2.2, 2.4, 2.4, 2.2; 3.7, 3.7, 3.9, 3.9];

    actSolution = intersect_lanelets(shape, lanelet);
    expSolution = true;
    verifyEqual(testcase, actSolution, expSolution)
end

function testPolygonPos(testcase)
    shape1 = [-7.0749 -2.8728 9.8889 21.3024 15.3469 7.9387
              -6.4707 -12.1152 -24.4428 -3.0950 19.3838 18.7030];
    shape2 = shape1 - [5; 5];
    actSolution = intersect_sat(shape1, shape2);
    expSolution = true;
    verifyEqual(testcase, actSolution, expSolution)
end

function testPolygonNeg(testcase)
    shape1 = [-7.0749 -2.8728 9.8889 21.3024 15.3469 7.9387
              -6.4707 -12.1152 -24.4428 -3.0950 19.3838 18.7030];
    shape2 = shape1 - [40; 40];
    actSolution = intersect_sat(shape1, shape2);
    expSolution = false;
    verifyEqual(testcase, actSolution, expSolution)
end
