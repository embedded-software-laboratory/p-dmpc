classdef ControllerTiming < handle

    properties (Access = private)
        controller_start_time (1, 1) uint64 % Start time of HLC

        name_to_timing_per_timestep (1, 1) dictionary % maps string to column in `timings`
        name_to_timing_once (1, 1) dictionary

        timings_per_timestep (2, :, :) double % start time and duration of timers per timestep
        timings_once (2, :) double % start time and duration of timers executed once
    end

    methods (Access = public)

        function obj = ControllerTiming()
            obj.name_to_timing_once = dictionary(string([]), uint32([]));
            obj.name_to_timing_per_timestep = dictionary(string([]), uint32([]));
            obj.controller_start_time = tic;
        end

        function start_timer(obj, name, timestep)

            arguments
                obj (1, 1) ControllerTiming
                name (1, 1) string
                timestep = [] % optional
            end

            if isempty(timestep)
                % create empty struct for timer start and elapsed time
                if ~isKey(obj.name_to_timing_once, name)

                    if isempty(values(obj.name_to_timing_once))
                        obj.name_to_timing_once(name) = 1;
                    else
                        obj.name_to_timing_once(name) = max(values(obj.name_to_timing_once)) + 1;
                    end

                end

                % set start time
                obj.timings_once(1, obj.name_to_timing_once(name)) = toc(obj.controller_start_time);
            else
                % create empty struct for timer start and elapsed time
                if ~isKey(obj.name_to_timing_per_timestep, name)

                    if isempty(values(obj.name_to_timing_per_timestep))
                        obj.name_to_timing_per_timestep(name) = 1;
                    else
                        obj.name_to_timing_per_timestep(name) = max(values(obj.name_to_timing_per_timestep)) + 1;
                    end

                end

                % set start time
                obj.timings_per_timestep(1, obj.name_to_timing_per_timestep(name), timestep) = toc(obj.controller_start_time);
            end

        end

        function elapsed_time = stop_timer(obj, name, timestep)

            arguments
                obj (1, 1) ControllerTiming
                name (1, 1) string
                timestep = [] % optional
            end

            if isempty(timestep)
                elapsed_time = toc(obj.controller_start_time) - obj.get_start_time(name);

                % save elapsed time
                obj.timings_once(2, obj.name_to_timing_once(name)) = elapsed_time;
            else
                elapsed_time = toc(obj.controller_start_time) - obj.get_start_time(name, timestep);

                % save elapsed time
                obj.timings_per_timestep(2, obj.name_to_timing_per_timestep(name), timestep) = elapsed_time;
            end

        end

        function start_time = get_start_time(obj, name, timestep)

            arguments
                obj (1, 1) ControllerTiming
                name (1, 1) string
                timestep = [] % optional
            end

            if isempty(timestep)

                if ~isKey(obj.name_to_timing_once, name)
                    error("no start time for specified timer name");
                end

                start_time = obj.timings_once(1, obj.name_to_timing_once(name));
            else

                if ~isKey(obj.name_to_timing_per_timestep, name)
                    error("no start time for specified timer name");
                end

                start_time = obj.timings_per_timestep(1, obj.name_to_timing_per_timestep(name), timestep);
            end

            if isempty(start_time) || (start_time == 0)
                error("start time for timer not set or set to zero");
            end

        end

        function elapsed_time = get_elapsed_time(obj, name, timestep)

            arguments
                obj (1, 1) ControllerTiming
                name (1, 1) string
                timestep = [] % optional
            end

            if isempty(timestep)

                if ~isKey(obj.name_to_timing_once, name)
                    error("no time elapsed for specified timer name");
                end

                elapsed_time = obj.timings_once(2, obj.name_to_timing_once(name));
            else

                if ~isKey(obj.name_to_timing_per_timestep, name)
                    error("no time elapsed for specified timer name");
                end

                elapsed_time = obj.timings_per_timestep(2, obj.name_to_timing_per_timestep(name), timestep);
            end

            if isempty(elapsed_time) || (elapsed_time == 0)
                error("elapsed time for timer not set or set to zero");
            end

        end

        function elapsed_times = get_all_elapsed_times(obj)
            elapsed_times = struct;

            % save results from timins_once
            timer_names = keys(obj.name_to_timing_once);

            for i = 1:length(timer_names)
                name = timer_names{i};
                elapsed_times.(name) = obj.timings_once(:, obj.name_to_timing_once(name));
            end

            % save results from timings_per_timestep
            timer_names = keys(obj.name_to_timing_per_timestep);

            for i = 1:length(timer_names)
                name = timer_names{i};
                elapsed_times.(name) = obj.timings_per_timestep(:, obj.name_to_timing_per_timestep(name), :);
            end

        end

    end

end
