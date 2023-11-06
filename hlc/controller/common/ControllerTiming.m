classdef ControllerTiming < handle

    properties (Access = private)
        controller_start_time (1, 1) uint64 % Start time of HLC

        names_of_timings_per_time_step (1, :) string % maps string to column in `timings`
        names_of_timings_once (1, :) string

        timings_per_time_step (2, :, :) double % start time and duration of timers per time step
        timings_once (2, :) double % start time and duration of timers executed once
    end

    methods (Access = public)

        function obj = ControllerTiming()
            obj.controller_start_time = tic;
        end

        function start(obj, name, time_step)

            arguments
                obj (1, 1) ControllerTiming
                name (1, 1) string
                time_step (1, 1) {mustBeInteger, mustBePositive} = 1 % optional
            end

            % create empty struct for timer start and elapsed time
            % set start time
            if nargin == 2
                column = obj.name_to_timing_once(name);
                obj.timings_once(1, column) = toc(obj.controller_start_time);
            else
                column = obj.name_to_timing_per_time_step(name);
                obj.timings_per_time_step(1, column, time_step) = toc(obj.controller_start_time);
            end

        end

        function elapsed_time = stop(obj, name, time_step)

            arguments
                obj (1, 1) ControllerTiming
                name (1, 1) string
                time_step (1, 1) {mustBeInteger, mustBePositive} = 1 % optional
            end

            assert(obj.timer_exists(name), "timer does not exist");

            end_time = toc(obj.controller_start_time);

            % save elapsed time
            if nargin == 2
                elapsed_time = end_time - obj.timings_once(1, obj.name_to_timing_once(name));
                obj.timings_once(2, obj.name_to_timing_once(name)) = elapsed_time;
            else
                elapsed_time = end_time - obj.timings_per_time_step(1, obj.name_to_timing_per_time_step(name), time_step);
                obj.timings_per_time_step(2, obj.name_to_timing_per_time_step(name), time_step) = elapsed_time;
            end

        end

        function elapsed_time = get_elapsed_time(obj, name, time_step)

            arguments
                obj (1, 1) ControllerTiming
                name (1, 1) string
                time_step (1, 1) {mustBeInteger, mustBePositive} = 1 % optional
            end

            assert(obj.timer_exists(name), "timer does not exist");

            % get elapsed time
            if nargin == 2
                elapsed_time = obj.timings_once(2, obj.name_to_timing_once(name));
            else
                elapsed_time = obj.timings_per_time_step(2, obj.name_to_timing_per_time_step(name), time_step);
            end

        end

        function timings = get_all_timings(obj)
            timings = struct;

            % save controller_start_time for normalization with other workers/computers in eval
            timings.controller_start_time = obj.controller_start_time;

            % save results from timings_once
            for name = obj.names_of_timings_once
                timings.(name) = obj.timings_once(:, obj.name_to_timing_once(name));
            end

            % save results from timings_per_time_step
            for name = obj.names_of_timings_per_time_step
                timings.(name) = obj.timings_per_time_step(:, obj.name_to_timing_per_time_step(name), :);
            end

        end

    end

    methods (Access = private)

        function column = name_to_timing_per_time_step(obj, name)
            column = find(strcmp(obj.names_of_timings_per_time_step, name), 1);

            if isempty(column)
                obj.names_of_timings_per_time_step(end + 1) = name;
                column = length(obj.names_of_timings_per_time_step);
            end

        end

        function column = name_to_timing_once(obj, name)
            column = find(strcmp(obj.names_of_timings_once, name), 1);

            if isempty(column)
                obj.names_of_timings_once(end + 1) = name;
                column = length(obj.names_of_timings_once);
            end

        end

        function tf = timer_exists(obj, name)
            tf = ~( ...
                isempty(find(strcmp(obj.names_of_timings_per_time_step, name), 1)) ...
                && isempty(find(strcmp(obj.names_of_timings_once, name), 1)) ...
            );
        end

    end

end
