classdef (Abstract) RunningEnv < handle
% SIMENV    Abstract class used for defining preoperties and mehtods used by different simulation setups.
    
    properties (Access=protected)
        % struct used for every iteration
        x0
        trim_indices
        cur_node
        controller
        scenario
        k
    end
    
    methods (Abstract) 
        setup(obj)
        [ x0, trim_indices ] = measure(obj)
        apply(obj, u, y_pred, info)
        got_stop = is_stop(obj)
        end_run(obj)
    end
end

