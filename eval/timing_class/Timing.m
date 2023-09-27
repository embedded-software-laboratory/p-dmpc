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
            % Initialise timers to map from timer names to
            % struct of start times and elapsed times
            new_obj.timers = dictionary(string([]), struct('start_time', uint64([]), 'elapsed_time', double([])));
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

            % create empty struct for timer start and elapsed time
            obj.timers(name) = struct('start_time', uint64([]), 'elapsed_time', double([]));
            % start timer
            obj.timers(name).start_time = tic;

            obj.set_singleton_data(obj);
        end

        function elapsed_time = stop_timer(name)
            obj = Timing.instance().get_singleton_data();

            if ~isempty(obj.timers(name).elapsed_time)
                warning("overwriting existing elapsed time");
            end

            elapsed_time = toc(obj.timers(name).start_time);

            % save elapsed time to dictionary
            obj.timers(name).elapsed_time = elapsed_time;
            obj.set_singleton_data(obj);
        end

        function elapsed_time = get_elapsed_time(name)
            obj = Timing.instance().get_singleton_data();
            % if timer does not exist, return error
            if ~isKey(obj.timers, name)
                error("get_elapsed_time called with non-existent timer, start timer first!");
            end

            % if not stopped yet, stop
            if isempty(obj.timers(name).elapsed_time)
                elapsed_time = Timing.stop_timer(name);
                return
            end

            % return elapsed time
            elapsed_time = obj.timers(name).elapsed_time;
        end

        function elapsed_times = get_all_elapsed_times()
            obj = Timing.instance().get_singleton_data();
            elapsed_times = dictionary(string([]), double([]));

            timer_names = keys(obj.timers);

            for i = 1:length(timer_names)
                name = timer_names{i};

                % if not stopped yet, stop the timer now
                if isempty(obj.timers(name).elapsed_time)
                    Timing.stop_timer(name);
                end

                elapsed_times(name) = obj.timers(name).elapsed_time;
            end

        end

    end

end
