classdef PriorityQueue
    properties (Constant, Hidden = true)
        NEW       = 0;
        DELETE    = 1;
        PUSH      = 2;
        POP       = 3;
        TOP       = 4;
        SIZE      = 5;
        TOP_VALUE = 6;
    end
    properties (SetAccess = private, Hidden = true)
        iObj; % Index to the underlying C++ class instance vector
    end
    methods
        function this = PriorityQueue()
            this.iObj = priority_queue_interface_mex(PriorityQueue.NEW);
        end

        function delete(this)
            priority_queue_interface_mex(PriorityQueue.DELETE, this.iObj);
        end

        function push(this, id, prio_val)
            priority_queue_interface_mex(PriorityQueue.PUSH, this.iObj, id, prio_val);
        end

        function [id,val] = pop(this)
            [id,val] = priority_queue_interface_mex(PriorityQueue.POP, this.iObj);
        end

        function [id,val] = top(this)
            [id,val] = priority_queue_interface_mex(PriorityQueue.TOP, this.iObj);
        end

        function result = size(this)
            result = priority_queue_interface_mex(PriorityQueue.SIZE, this.iObj);
        end

        function result = top_value(this, varargin)
            result = priority_queue_interface_mex(PriorityQueue.TOP_VALUE, this.iObj);
        end
    end
end