%% Add modules to path
% Import tree class
assert(logical(exist('./matlab-tree/', 'dir')));
addpath('./matlab-tree/');
addpath(genpath(pwd));
warning('off','MATLAB:polyshape:repairedBySimplify')

angles = selection();

scenario = run_simulation(angles);

%% Log workspace to subfolder 
st = dbstack;
namestr = st(1).name;
sub_folder = './logs/' + string(namestr);
file_name = fullfile(sub_folder,'data');
fig_name = fullfile(sub_folder,'fig');

if ~exist(sub_folder, 'dir')
    mkdir(sub_folder)
end

% Get a list of all variables
allvars = whos;

% Identify the variables that ARE NOT graphics handles. This uses a regular
% expression on the class of each variable to check if it's a graphics object
tosave = cellfun(@isempty, regexp({allvars.class}, '^matlab\.(ui|graphics)\.'));

% Pass these variable names to save
save(file_name, allvars(tosave).name)
savefig(fig_name);