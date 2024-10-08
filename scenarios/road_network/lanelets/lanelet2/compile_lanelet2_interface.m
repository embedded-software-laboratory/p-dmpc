function compile_lanelet2_interface
    folder = fileparts(mfilename('fullpath'));
    is_load_lanelet2_map_matlab_mex_compiled = isequal(fileparts(which('load_lanelet2_map_matlab_mex')), folder);
    is_generate_lanelet2_reference_path_loop_indices_mex_compiled = isequal(fileparts(which('generate_lanelet2_reference_path_loop_indices_mex')), folder);
    is_generate_lanelet2_rpss_indices_mex_compiled = isequal(fileparts(which('generate_lanelet2_ref_path_separate_segments_indices_mex')), folder);

    cwd = cd(folder);
    cleanup_obj = onCleanup(@() cd(cwd));
    fprintf('Check whether lanelet2 mex functions need to be compiled...\n');

    if ~is_load_lanelet2_map_matlab_mex_compiled
        fprintf('Compiling load_lanelet2_map_matlab_mex... ');
        mex "CXXFLAGS='$CXXFLAGS -std=c++17'" ...
            "load_lanelet2_map_matlab_mex.cpp" ...
            "Rosless-Lanelet2/lanelet2_projection/src/CPM.cpp" ...
            "-I/usr/include/eigen3" ...
            "-IRosless-Lanelet2/lanelet2_projection/include" ...
            "-llanelet2_io" ...
            "-llanelet2_core" ...
            "-llanelet2_routing" ...
            "-llanelet2_traffic_rules"
        fprintf('Done.\n');
    else
        fprintf('Already compiled: load_lanelet2_map_matlab_mex.\n')
    end

    if ~is_generate_lanelet2_reference_path_loop_indices_mex_compiled
        fprintf('Compiling generate_lanelet2_reference_path_loop_indices_mex...');
        mex "CXXFLAGS='$CXXFLAGS -std=c++17'" ...
            "generate_lanelet2_reference_path_loop_indices_mex.cpp" ...
            "Rosless-Lanelet2/lanelet2_projection/src/CPM.cpp" ...
            "-I/usr/include/eigen3" ...
            "-IRosless-Lanelet2/lanelet2_projection/include" ...
            "-llanelet2_io" ...
            "-llanelet2_core" ...
            "-llanelet2_routing" ...
            "-llanelet2_traffic_rules"
        fprintf('Done.\n');
    else
        fprintf('Already compiled: generate_lanelet2_reference_path_loop_indices_mex.\n')
    end

    if ~is_generate_lanelet2_rpss_indices_mex_compiled
        fprintf('Compiling generate_lanelet2_ref_path_separate_segments_indices_mex.\n...');
        mex "CXXFLAGS='$CXXFLAGS -std=c++17'" ...
            "generate_lanelet2_ref_path_separate_segments_indices_mex.cpp" ...
            "Rosless-Lanelet2/lanelet2_projection/src/CPM.cpp" ...
            "-I/usr/include/eigen3" ...
            "-IRosless-Lanelet2/lanelet2_projection/include" ...
            "-llanelet2_io" ...
            "-llanelet2_core" ...
            "-llanelet2_routing" ...
            "-llanelet2_traffic_rules"
        fprintf('Done.\n');
    else
        fprintf('Already compiled: generate_lanelet2_ref_path_separate_segments_indices_mex.\n')
    end

end
