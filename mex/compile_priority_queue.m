function compile_priority_queue
    %MATLAB crahes if object of PriorityQueue exists when compiling!
    folder = fileparts(mfilename('fullpath'));
    cwd = cd(folder);
    cleanup_obj = onCleanup(@() cd(cwd));
    fprintf('Compiling priority_queue_interface_mex\n');
    mex priority_queue_interface_mex.cpp
end