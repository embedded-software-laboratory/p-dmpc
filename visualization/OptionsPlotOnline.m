classdef OptionsPlotOnline
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        is_active = true;                       % is online plot active
        plot_vehicle_id = true;                 % whether to show vehicle IDs
        plot_priority = true;                   % whether to show priority colorbar
        plot_coupling = true;                   % whether to show coupling edges
        plot_weight = true;                     % whether to show coupling weights
        plot_hotkey_description = false;        % whether to show description of hotkeys
        plot_reachable_sets = false;            % whether to show reachable sets
        vehicles_reachable_sets = [];           % reachable sets of those vehicles will be shown
        plot_lanelet_crossing_areaas = false;   % whether to show lanelet crossing areas
        vehicles_lanelet_crossing_areas = [];   % lanelet corssing area of those vehicles will be shown
        plot_predicted_occupancy = false;       % whether to show predicted occupancy
        vehicles_predicted_occupancy = [];      % predicted occupancy of those vehicles will be shown
        plot_predicted_occupancy_previous = false;      % whether to show previously predicted occupancy
        vehicles_predicted_occupancy_previous = [];     % previously predicted occupancy of those vehicles will be shown
        is_video_mode = false;                          % if in video mode, the lanelets need to be plotted for each time step
        plot_scenario_name = true;  % whether to show scenario name in the figure title
        plot_timesteps = true;      % whether to show timesteps in the figure title
        is_custom_colors = false;   % whether to use custom colors to visualize vehicles
        custom_colors = [];         % custom colors (nVeh-by-3 matrix)
        is_dynamic_colors = true;   % whether to use dynamic colors to visualize vehicles (i.e, the colors of vehicles change dynamically with their priorities)
        plot_xy_labels = true;      % whether plot x- and y-axis labels
        plot_xy_ticks = true;       % whether plot x- and y-axis ticks
    end

    methods
        function obj = OptionsPlotOnline()
        end

        function obj = assign_data(obj,struct)
            fn = fieldnames(struct);
            for i_field = 1:length(fn)
                field = fn{i_field};
                if ~isprop(obj,field)
                    warning('Cannot set property %s for class OptionsPlotOnline as it does not exist', field);
                    continue;
                end
                obj.(field) = struct.(field);
            end
        end
    end
end