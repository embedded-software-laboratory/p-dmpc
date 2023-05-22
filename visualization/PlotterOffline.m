classdef PlotterOffline < Plotter
    %PLOTTEROFFLINE Plotter class for offline result analysis.
    %   From a results object, the different time steps can be plotted and iterated forwards and backwards.

    properties
        result (1, 1) struct % results to plot
    end

    properties (Access = private)
        delta_t_s (1, 1) double % time step duration for playback
    end

    methods

        function obj = PlotterOffline(result, delta_t_s, veh_indices)
            %PLOTTEROFFLINE Construct an instance of PlotterOffline
            %   Specify the results to view and optionally the time step for playback and the vehicle indices to plot.
            arguments
                result (1, 1) struct
                delta_t_s (1, 1) double = result.scenario.options.dt
                veh_indices (1, :) int32 = 1:result.scenario.options.amount
            end

            obj@Plotter(result.scenario, veh_indices);
            obj.result = result;
            obj.delta_t_s = delta_t_s;
            obj.paused = true;
            obj.hotkey_description(obj.number_base_hotkeys) = sprintf("{\\its}: change playback speed (%0.3gs)", delta_t_s);
            obj.hotkey_description(obj.number_base_hotkeys + 5) = '';
            obj.hotkey_description(obj.number_base_hotkeys + 6) = "{\itesc}: abort";
            set(obj.fig, 'WindowKeyPressFcn', @obj.keyPressCallback);
        end

        function start(obj)
            %START Start plotting.
            %   Show the first time step and enable playback and manual iteration through the results.

            obj.plot();
            % Keep plotter alive until plotting is aborted.
            while ~obj.abort
                pause(obj.delta_t_s);
                % Pause playback on last time step.
                if ~obj.paused && obj.time_step == obj.result.nSteps
                    obj.paused = true;
                    obj.update_hotkey_description();
                    obj.plot();
                    disp('Pausing playback as last time step is reached.')
                end

                % Plot the next time step during playback.
                if ~obj.paused
                    obj.time_step = obj.time_step + 1;
                    obj.plot();
                end

            end

        end

        function plot(obj)
            %PLOT  Plot the simulation state at the current time step.

            plotting_info = PlottingInfo(obj.veh_indices, obj.result, obj.time_step, 1);
            plot@Plotter(obj, plotting_info);
        end

    end

    methods (Access = protected)

        function update_hotkey_description(obj, initial)
            %UPDATE_HOTKEY_DESCRIPTION
            %   Set the hotkey descriptions depending on if the hot key is currently active or inactive.
            arguments
                obj PlotterOffline
                initial logical = false
            end

            if ~initial
                obj.hotkey_description(obj.number_base_hotkeys) = sprintf("{\\its}: change playback speed (%0.3gs)", obj.delta_t_s);
            end

            if ~initial && obj.time_step > 1
                obj.hotkey_description(obj.number_base_hotkeys + 1) = "{\color{black}{\itleftarrow}: 1 time step backward}";
                obj.hotkey_description(obj.number_base_hotkeys + 3) = "{\color{black}{\itpos1}: jump to first time step}";
            else
                obj.hotkey_description(obj.number_base_hotkeys + 1) = "{\color{gray}{\itleftarrow}: 1 time step backward}";
                obj.hotkey_description(obj.number_base_hotkeys + 3) = "{\color{gray}{\itpos1}: jump to first time step}";
            end

            if initial || obj.time_step < obj.result.nSteps
                obj.hotkey_description(obj.number_base_hotkeys + 2) = "{\color{black}{\itrightarrow}: 1 time step forward}";
                obj.hotkey_description(obj.number_base_hotkeys + 4) = "{\color{black}{\itend}: jump to last time step}";
            else
                obj.hotkey_description(obj.number_base_hotkeys + 2) = "{\color{gray}{\itrightarrow}: 1 time step forward}";
                obj.hotkey_description(obj.number_base_hotkeys + 4) = "{\color{gray}{\itend}: jump to last time step}";
            end

            update_hotkey_description@Plotter(obj);

            if obj.paused && obj.time_step == obj.result.nSteps
                obj.hotkey_description(11) = "{\color{gray}{\itspace}: start simulation}";
            end

        end

        function plot_hotkey_description(obj)
            %PLOT_HOTKEY_DESCRIPTION
            %   Plot the hotkey descriptions next to the scenario plot, specific for 1- or 2-circle scenario, else
            %   general plotting routine.

            if obj.scenario.options.scenario_type == ScenarioType.circle && obj.scenario.options.amount <= 2
                position = obj.hotkey_position;
                text(position(1), position(2), obj.hotkey_description(1:obj.number_base_hotkeys - 2), ...
                    'FontSize', 12, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', 'Tag', 'hotkey');
                text(position(1) + 1.2, position(2), obj.hotkey_description(obj.number_base_hotkeys - 2:end), ...
                    'FontSize', 12, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', 'Tag', 'hotkey');
            else
                plot_hotkey_description@Plotter(obj);
            end

        end

        function keyPressCallback(obj, ~, eventdata)
            %KEY_PRESS_CALLBACK Callback function for PlotterOffline specific figure key press event.
            %   Enable/disable plotting options on hot key press and replot.
            arguments
                obj (1, 1) Plotter
                ~
                eventdata (1, 1) matlab.ui.eventdata.KeyData
            end

            switch eventdata.Key
                case 'rightarrow'

                    if obj.time_step < obj.result.nSteps
                        obj.time_step = obj.time_step + 1;
                    else
                        fprintf("Cannot increase time step as the maximum time step of %i is reached.\n", obj.result.nSteps);
                    end

                case 'leftarrow'

                    if obj.time_step > 1
                        obj.time_step = obj.time_step - 1;
                    else
                        fprintf("Cannot decrease time step as the minimum time step of 1 is reached.\n");
                    end

                case 'home'
                    obj.time_step = 1;
                    disp("Jumping to first time step.")
                case 'end'
                    obj.time_step = obj.result.nSteps;
                    disp("Jumping to last time step.")
                case 's'
                    new_delta_t_s = input("Enter a new time step playback duration in seconds (s to cancel):\n", "s");
                    new_delta_t_s = strrep(new_delta_t_s, ',', '.');

                    while isnan(str2double(new_delta_t_s)) && ~strcmp(new_delta_t_s, "s")
                        new_delta_t_s = input("Invalid input. Enter a new time step playback duration (s to cancel):\n", "s");
                        new_delta_t_s = strrep(new_delta_t_s, ',', '.');
                    end

                    if ~isnan(str2double(new_delta_t_s))
                        obj.delta_t_s = str2double(new_delta_t_s);
                    end

            end

            keyPressCallback@Plotter(obj, [], eventdata);
            obj.plot();
        end

    end

end
