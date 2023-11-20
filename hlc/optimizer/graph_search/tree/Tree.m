classdef Tree < handle

    properties (SetAccess = protected)
        parent (1, :) uint32; % Index of the parent node
    end

    methods (Abstract)

        get_node(obj, i_node)
        size(obj)

        number_of_vehicles(obj)

        get_x(obj, i_vehicle, i_node)
        get_y(obj, i_vehicle, i_node)
        get_yaw(obj, i_vehicle, i_node)
        get_trim(obj, i_vehicle, i_node)

    end

    methods

        function i_node = get_parent(obj, i_node)
            i_node = obj.parent(i_node);
        end

        function result = path_to_root(obj, i_node)
            %% PATH_TO_ROOT  Path from node i_node to the Tree root.
            result = i_node;

            while result(end) ~= 1
                result(end + 1) = obj.parent(result(end)); %#ok<AGROW>
            end

        end

    end

end
