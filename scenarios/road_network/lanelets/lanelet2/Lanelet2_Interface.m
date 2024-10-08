classdef Lanelet2_Interface

    methods (Static)

        function [raw_data, intersection_lanelets, ids] = load_from_file(filename)
            % filename: path to .osm file containing a lanelet2 map
            % raw_data: a struct as needed for RoadDataCommonRoad
            %           representing the given lanelet2 map
            % intersection_lanelets: An array containing all (matlab)
            %           indices of lanelets which are part of an intersection
            % ids:      an array giving at (matlab) index i the
            %           lanelet2 ID of the corresponding lanelet
            [~, ~, intersection_lanelets, ids, ~, raw_data] ...
                = load_lanelet2_map_matlab_mex(convertCharsToStrings(filename));
        end

        function indices = generate_reference_path_indices(filename)
            % filename: path to .osm file containing a lanelet2 map
            % return: an array of (matlab) indices defining a random cycle
            %         on the given map
            indices = generate_lanelet2_reference_path_loop_indices_mex(convertCharsToStrings(filename));
        end

        function indices = generate_lanelet2_ref_path_separate_segments_indices(filename, segment_ids)
            % filename: path to .osm file containing a lanelet2 map
            % segment_ids: ids of all segments which shall be covered in
            %              the corresponding order
            % return: an array of (matlab) indices defining a path covering
            %         all the given points
            indices = generate_lanelet2_ref_path_separate_segments_indices_mex(convertCharsToStrings(filename), int64(segment_ids));
        end

    end

end
