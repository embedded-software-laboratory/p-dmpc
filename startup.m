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
end
