classdef Timing < Singleton
    %   Template written by Bobby Nedelkovski
    %   The MathWorks Australia Pty Ltd
    %   Copyright 2009, The MathWorks, Inc.

    properties % Public Access
        timers (1, 1) dictionary
    end

    methods (Access = private)
        % Guard the constructor against external invocation.  We only want
        % to allow a single instance of this class.  See description in
        % Singleton superclass.
        function new_obj = Timing()
            % Initialise timers to map from timer names to return type of tic function
            new_obj.timers = dictionary(string([]), uint64([]));
        end

    end

    methods (Static)

        function obj = instance()
            persistent unique_instance

            if isempty(unique_instance)
                obj = Timing();
                unique_instance = obj;
            else
                obj = unique_instance;
            end

        end

        function start_timer(name)
            fprintf("start_timer called with name %s \n", name);
            obj = Timing.instance();

            % start timer
            obj.timers(name) = tic;

            obj.set_singleton_data(obj);
        end

        function time = stop_timer(name)
            obj = Timing.instance().get_singleton_data();
            time = toc(obj.timers(name));
            fprintf("stop_timer called with name %s, have obj.timers:\n", name);
            disp(obj.timers);
        end

    end

end
