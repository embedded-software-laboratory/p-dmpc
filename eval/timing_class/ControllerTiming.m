classdef ControllerTiming < handle

    properties (Access = private)
        timers (1, 1) dictionary
    end

    methods (Access = public)
        function obj = ControllerTiming
            obj.timers = dictionary(string([]), struct('start_time', uint64([]), 'elapsed_time', double([])));
        end

        function start_timer(obj, name)
            % create empty struct for timer start and elapsed time
            obj.timers(name) = struct('start_time', uint64([]), 'elapsed_time', double([]));
            % start timer
            obj.timers(name).start_time = tic;
        end

        function elapsed_time = stop_timer(obj, name)
            if ~isempty(obj.timers(name).elapsed_time)
                warning("Overwriting existing elapsed time. Use timer series for multiple values");
            end

            elapsed_time = toc(obj.timers(name).start_time);

            % save elapsed time to dictionary
            obj.timers(name).elapsed_time = elapsed_time;

            fprintf("stop_timer called for %s, time %f, have obj.timers:\n", name, elapsed_time);
            disp(obj.timers);
        end

        function elapsed_time = get_elapsed_time(obj, name)
            % if timer does not exist, return error
            if ~isKey(obj.timers, name)
                error("get_elapsed_time called with non-existent timer, start timer first!");
            end

            % if not stopped yet, stop
            if isempty(obj.timers(name).elapsed_time)
                elapsed_time = obj.stop_timer(name);
                return
            end

            % return elapsed time
            elapsed_time = obj.timers(name).elapsed_time;
        end

        function elapsed_times = get_all_elapsed_times(obj)
            elapsed_times = dictionary(string([]), double([]));

            timer_names = keys(obj.timers);

            for i = 1:length(timer_names)
                name = timer_names{i};

                % if not stopped yet, stop the timer now
                if isempty(obj.timers(name).elapsed_time)
                    obj.stop_timer(name);
                end

                elapsed_times(name) = obj.timers(name).elapsed_time;
            end

        end

    end

end
