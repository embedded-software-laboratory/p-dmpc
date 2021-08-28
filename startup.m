function startup()
% STARTUP   Startup function. Call before executing the main function.

if ispc % windows
    splitchar = ';';
else
    splitchar = ':';
end
% Add modules to path
dirs = strsplit(genpath(pwd),splitchar);
ignore_dirs = ...
    {'output'...
    ,'.git'...
};
for d=dirs
    if(~contains(d{1},ignore_dirs))
        addpath(d{1});
    end
end



folder = fullfile(fileparts(mfilename('fullpath')),'mex');
if ~isequal(fileparts(which('priority_queue_interface_mex')), folder)
    % Compile the mex
    cwd = cd(folder);
    cleanup_obj = onCleanup(@() cd(cwd));
    fprintf('Compiling priority_queue_interface_mex\n');
    mex priority_queue_interface_mex.cpp
end

end
