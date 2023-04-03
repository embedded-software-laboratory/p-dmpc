function result = read_object_from_input(variable_input, classname)
    %read_object_from_input finds an object ob the class `classname` in
    %varargin
    is_object = cellfun('isclass', variable_input, classname);

    if sum(is_object) == 1
        result = variable_input{is_object};
    else
        result = [];
    end

end
