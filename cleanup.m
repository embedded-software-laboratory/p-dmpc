function cleanup()
    % This function cleans all generated files. This is useful, e.g., when
    % changing anything w.r.t. the compilers or when upgrading the matlab
    % version.
    rmdir_helper("commun/cust1/matlab_msg_gen");
    rmdir_helper("commun/cust2/matlab_msg_gen");
    rmdir_helper("commun/unified_lab_api/matlab_msg_gen");
    rmfile_helper("lanelets/lanelet2/generate_lanelet2_ref_path_loop_indices_mex.mexa64");
    rmfile_helper("lanelets/lanelet2/load_lanelet2_map_matlab_mex.mexa64");
    rmfile_helper("mex/priority_queue_interface_mex.mexa64");
    compile_priority_queue;
end


function rmdir_helper(dir)
    try
        rmdir(dir, 's');
    catch
        warning(strcat("Unable to delete folder ",dir,". Please delete manually..."));
    end
end

function rmfile_helper(file)
    try
        delete(file);
    catch
        warning(strcat("Unable to delete file ",file,". Please delete manually..."));
    end
end