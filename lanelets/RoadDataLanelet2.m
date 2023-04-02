classdef RoadDataLanelet2 < RoadDataCommonRoad
    %ROADDATALANELET2
    %   This class can load Lanelet2 data. Since CommonRoad and Lanelet2
    %   are quite similar, this class is a subclass of RoadDataCommonRoad
    %   such that it can reuse much funcionality.
    
    properties
        map_as_string   % The whole file read as string. This is needed for sending the map via ULA to a lab.
    end
    
    methods (Access=protected)
        function default_road_name = get_default_road_name(~)
            default_road_name = 'lab_reduced.osm';
        end

        function road_data = compute_road_data(road_data)
            % Implements abstract method in superclass. Method will be
            % called from constructor to construct whole data if not
            % already existing

            % read raw road data: Use c++ lanelet2 library via mex
            [road_data.road_raw_data.lanelet, road_data.intersection_lanelets, road_data.road_raw_data.id_mapping] ...
                = Lanelet2_Interface.load_from_file([road_data.road_folder_path,filesep,road_data.road_name]);
            
            % get lanelet coordinates
            road_data.lanelets = road_data.get_lanelets();
        
            % Note: The CommonRoad method for getting the intersection lanelets
            % cannot be reused, so the intersecetion lanelets are already read
            % via mex above.

            % get adjacent lanelet relationships with `adjacency_lanelets` as byproduct
            [road_data.lanelet_relationships,road_data.adjacency_lanelets] = road_data.get_lanelet_relationships();
        
            % get lanelet boundaries
            [road_data.lanelet_boundary,share_boundary_with] = road_data.get_lanelet_boundary();

            % find adjacent lanelets according to whether their lanelet boundaries intersect
            road_data = road_data.get_adjacent_lanelets();

            % update lanelet relationships
            road_data = road_data.update_lanelet_relationships(share_boundary_with);

            
            % Additionally to the previous steps, read the file now as string.
            road_data.map_as_string = fileread([road_data.road_folder_path,filesep,road_data.road_name]);
        end
    end
end