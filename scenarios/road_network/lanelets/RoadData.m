classdef (Abstract) RoadData
    %ROADDATA Summary of this class goes here
    %   Detailed explanation goes here

    %     properties(Abstract,Constant)
    %         default_road_name       % Default road name. Should be set by each subclass
    %     end

    properties
        lanelets % 1-by-nLanelets cell array, stores the coordinates of left bound, right bound, and centerline of each lanelet
        adjacency_lanelets % nLanelets-by-nLanelets matrix. Entry is true iff two lanelets are adjacent
        intersection_lanelets % indices of lanelets of intersections
        lanelet_boundary % 1-by-nLanelet cell array with each subcell being a 1-by-3 cell array. The first and second subcells store the left and right boundary of each lanelet.
        % The third subcell stores an object of MATLAB class `polyshape`.
        road_raw_data % road data before preprocessing
        lanelet_relationships % nLanelet-by-nLanelets cell array, stores the relationship of two adjacent lanelets. Empty if they are adjacent

        road_name % road name (no file ending)
        road_folder_path % path to folder where offline data is/should be stored
        %         road_full_path          % road_folder_path + road_name

        load_stored = true % stores whether a configuration should be loaded if existing
    end

    methods (Abstract, Access = protected)
        obj = compute_road_data(obj) % Called by constructor and loads all the data if no .mat already exists.
        s = get_default_road_name(obj) % Returns default road name. Should be set by each subclass
    end

    methods

        function road_data = RoadData(load_stored)

            if nargin == 0
                road_data.load_stored = true;
            else
                road_data.load_stored = load_stored;
            end

        end

        function road_data = get_road_data(road_data, road_name, road_path)
            % road_name: Name of the file without ending. If empty: default is used
            % folder_path: path to folder where the offline data is/should be stored

            % 1. Initialize road_name (default if not specified in constructor call)
            if nargin == 1
                road_data.road_name = road_data.get_default_road_name();
            else
                assert(isstring(road_name) || ischar(road_name))
                road_data.road_name = road_name;
            end

            % 2. Initialize road_path path of the road data
            if nargin <= 2 % path not specified. Use current location.
                [file_path, ~, ~] = fileparts(mfilename('fullpath'));
                road_data.road_folder_path = [file_path, filesep, 'offline_road_data'];
            else
                assert(isstring(road_path) || ischar(road_path))
                road_data.road_folder_path = road_path;
            end

            if ~isfolder(road_data.road_folder_path)
                % create target folder if not exist
                mkdir(road_data.road_folder_path)
            end

            road_full_path = [road_data.road_folder_path, filesep, road_data.road_name, '.mat'];

            %% if the needed road data alread exist, simply load it, otherwise they will be calculated and saved.
            if isfile(road_full_path) && road_data.load_stored
                load(road_full_path, 'road_data');
            else
                %% else load and preprocess the raw data and save them at the end
                road_data = road_data.compute_road_data();
                % save all the road data
                save(road_full_path, 'road_data', '-mat')
            end

        end

        function plot(obj)
            %% visualize lanelet
            % get road data
            road_data = obj.get_road_data();

            %% plot lanelets
            figure()
            plotRoadSetup;

            plot_lanelets(road_data.road_raw_data.lanelet, ScenarioType.commonroad);

            %% plot lanelet boundaries
            figure()
            plotRoadSetup;
            lanelet_boundary_poly = cellfun(@(c)[c{3}], road_data.lanelet_boundary);
            plot_lanelets(road_data.road_raw_data.lanelet, ScenarioType.commonroad);
            plot(lanelet_boundary_poly)

            %% local function
            function plotRoadSetup()
                % Setup of plot()
                hold on
                box on
                axis equal

                xlabel('\fontsize{14}{0}$x$ [m]', 'Interpreter', 'LaTex');
                ylabel('\fontsize{14}{0}$y$ [m]', 'Interpreter', 'LaTex');

                xlim([0, 4.5]);
                ylim([0, 4]);
                daspect([1 1 1])
            end

        end

    end

end
