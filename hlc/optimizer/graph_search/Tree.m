classdef Tree < handle

    properties (SetAccess = protected)
        % Index of the parent node. The root of the AStarTree as a parent index
        % equal to 0.
        parent (1, :) uint32;
    end

    methods (Abstract)

        get_node(obj, ID)
        size(obj)

        number_of_vehicles(obj)

        get_x(obj, i_vehicle, ID)
        get_y(obj, i_vehicle, ID)
        get_yaw(obj, i_vehicle, ID)
        get_trim(obj, i_vehicle, ID)

    end

    methods

        function ID = get_parent(obj, ID)
            %% GET_PARENT  Return the parent of the given node.
            ID = obj.parent(ID);
        end

        function result = path_to_root(obj, ID)
            %% PATH_TO_ROOT  Path from node ID to the Tree root.
            result = ID;

            while result(end) ~= 1
                result(end + 1) = obj.parent(result(end)); %#ok<AGROW>
            end

        end

    end

end
