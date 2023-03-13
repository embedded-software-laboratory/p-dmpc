function tests = test_read_object_from_input
    tests = functiontests(localfunctions);
end

function TEST_WITH_class_object_SHOULD_return_object(testcase)
    % define local function
    function result = return_object(classname, varargin)
        result = read_object_from_input(varargin,classname);
    end
    
    s = Scenario();
    
    class_name = 'Scenario';
    actual = return_object(class_name, s, magic(3));
    verifyInstanceOf(testcase,actual,class_name);
    verifyNotEmpty(testcase,actual);
end

function TEST_WITH_no_class_object_SHOULD_return_false(testcase)
    % define local function
    function result = return_object(classname, varargin)
        result = read_object_from_input(varargin,classname);
    end
    
    class_name = 'Scenario';
    actual = return_object(class_name, 1, magic(3));
    verifyEmpty(testcase,actual,class_name);
end