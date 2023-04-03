function compile_priority_queue
    %MATLAB crahes if object of PriorityQueue exists when compiling!
    folder = fileparts(mfilename('fullpath'));
    is_mex_compiled = isequal(fileparts(which('priority_queue_interface_mex')), folder);

    if ~is_mex_compiled
        cwd = cd(folder);
        cleanup_obj = onCleanup(@() cd(cwd));
        fprintf('Compiling priority_queue_interface_mex\n');
        mex priority_queue_interface_mex.cpp
    end

end
