function startup()
% Add modules to path
dirs = strsplit(genpath(pwd),';');
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