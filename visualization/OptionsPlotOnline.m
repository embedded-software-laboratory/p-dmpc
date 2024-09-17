classdef OptionsPlotOnline
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        is_active = true; % is online plot active
        plot_vehicle_id = true; % whether to show vehicle IDs
        plot_priority = true; % whether to show priority colorbar
        plot_coupling = true; % whether to show coupling edges
        plot_weight = true; % whether to show coupling weights
        plot_hotkey_description = false; % whether to show description of hotkeys
        plot_reachable_sets = false; % whether to show reachable sets
        vehicles_reachable_sets = []; % reachable sets of those vehicles will be shown
        is_video_mode = false; % if in video mode, the lanelets need to be plotted for each time step
    end

    methods

        function obj = OptionsPlotOnline()
        end

        function obj = jsondecode(obj, json_struct)
            % for each loop requires fields as row vector
            fields = string(fieldnames(json_struct)).';

            for field = fields

                if ~isprop(obj, field)
                    warning('Cannot set property %s for class OptionsPlotOnline as it does not exist', field);
                    continue;
                end

                obj.(field) = json_struct.(field);
            end

        end

    end

end
