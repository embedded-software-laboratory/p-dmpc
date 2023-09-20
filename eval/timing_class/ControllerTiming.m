classdef ControllerTiming < handle

    properties (Access = private)
        timers (1, 1) dictionary
        timer_series (1, 1) dictionary
    end

    methods (Access = public)
        function obj = ControllerTiming()
            obj.timers = dictionary(string([]), struct('start_time', uint64([]), 'elapsed_time', double([])));
            obj.timer_series = dictionary(string([]), struct('start_time', uint64([]), 'elapsed_time', double([])));
        end

        function start_timer(obj, name, timestep)
            arguments
                obj (1,1) ControllerTiming
                name (1,1) string
                timestep = [] % optional
            end

            if isempty(timestep)
                % create empty struct for timer start and elapsed time
                obj.timers(name) = struct('start_time', uint64([]), 'elapsed_time', double([]));
                % start timer
                obj.timers(name).start_time = tic;
            else
                if ~isKey(obj.timer_series, name)
                    % create empty struct for timer start and elapsed time
                    obj.timer_series(name) = struct('start_time', uint64([]), 'elapsed_time', double([]));
                end
                % start timer
                obj.timer_series(name).start_time(timestep) = tic;
            end
        end

        function elapsed_time = stop_timer(obj, name, timestep)
            arguments
                obj (1,1) ControllerTiming
                name (1,1) string
                timestep = [] % optional
            end

            if isempty(timestep)
                if ~isempty(obj.timers(name).elapsed_time)
                    warning("Overwriting existing elapsed time. Specify timestep for multiple values with same timer name.");
                end

                elapsed_time = toc(obj.timers(name).start_time);

                % save elapsed time to dictionary
                obj.timers(name).elapsed_time = elapsed_time;
            else
                elapsed_time = toc(obj.timer_series(name).start_time(timestep));

                % save elapsed time to dictionary
                obj.timer_series(name).elapsed_time(timestep) = elapsed_time;
            end

        end

        function elapsed_time = get_elapsed_time(obj, name, timestep)
            arguments
                obj (1,1) ControllerTiming
                name (1,1) string
                timestep = [] % optional
            end

            if isempty(timestep)
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
            else
                % TODO
                elapsed_time = 0;
            end

        end

        function elapsed_times = get_all_elapsed_times(obj)
            elapsed_times = struct;
            elapsed_times.timer_results = dictionary(string([]), double([]));
            elapsed_times.timer_series_results = dictionary(string([]), struct('per_timestep_values', double([])));

            if isConfigured(obj.timers)
                timer_names = keys(obj.timers);

                for i = 1:length(timer_names)
                    name = timer_names{i};

                    % if not stopped yet, stop the timer now
                    if isempty(obj.timers(name).elapsed_time)
                        obj.stop_timer(name);
                    end

                    elapsed_times.timer_results(name) = obj.timers(name).elapsed_time;
                end
            end

            if isConfigured(obj.timer_series)
                timer_names = keys(obj.timer_series);

                for i = 1:length(timer_names)
                  timer_name = timer_names{i};
                  amount_timesteps_recorded = length(obj.timer_series(timer_name).elapsed_time);
                  timer_results = double([]);

                  for timestep = 1:amount_timesteps_recorded
                      % if not stopped yet, stop the timer now
                      if isempty(obj.timer_series(timer_name).elapsed_time(timestep))
                          obj.stop_timer(timer_name, timestep);
                      end

                      timer_results(timestep) = obj.timer_series(timer_name).elapsed_time(timestep);
                  end
                  elapsed_times.timer_series_results(timer_name) = struct('per_timestep_values', double([]));
                  elapsed_times.timer_series_results(timer_name).per_timestep_values = timer_results;
                end
            end
        end

    end

end
