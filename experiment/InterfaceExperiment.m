classdef (Abstract) InterfaceExperiment < handle
% INTERFACEEXPERIMENT    Abstract class used for defining properties and methods used by different experiment setups.
    
    properties (Access=protected)
        % struct used for every iteration
        x0
        trim_indices
        cur_node
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

