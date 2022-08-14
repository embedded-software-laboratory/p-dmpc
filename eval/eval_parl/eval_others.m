%% print vehicle initial positions to fig
file_name = 'vehInitialPositions.mat';

[file_path,~,~] = fileparts(mfilename('fullpath')); % get the path of the current file
idcs = strfind(file_path,filesep); % find all positions of '/'
two_folders_up = file_path(1:idcs(end-1)-1); % two folders up
full_path = fullfile(two_folders_up,'results','Commonroad_RHC-Parl',file_name);

if isfile(full_path)
    disp('File already exists.')
    load(full_path,'result')
else
    % prepare simulation options
    options = OptionsMain;
    options.consider_RSS = false;
    options.is_sim_lab = true;
    options.customResultName = '';
    options.scenario = 'Commonroad';
    options.trim_set = 9;
    options.Hp = 5;
    options.dt = 0.2;
    options.T_end = 1;
    options.isPB = true;
    options.isParl = true;
    options.isAllowInheritROW = true;
    options.max_num_CLs = 4;
    options.strategy_consider_veh_without_ROW = '3';
    options.strategy_enter_lanelet_crossing_area = '4';
    options.isSaveResult = true;
    options.visu = [false,false];
    options.is_eval = false;
    options.visualize_reachable_set = false;
    options.priority = 'right_of_way_priority';
    options.amount = 35;

    % run simulation
    if exist('scenario','var')
        [result,~,~] = main(options,scenario);
    else
        [result,scenario,~] = main(options);
    end
end

set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');

set(0,'DefaultTextFontsize', 7)
set(0,'DefaultAxesFontsize', 7)

fig_x = 7;     fig_y = 7; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'vehInitialPositions';

fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [x_margin y_margin fig_x_position fig_y_position])
t_fig = tiledlayout(1,1,'Padding','compact','TileSpacing','compact');

% resolution = [1920 1080];
% fig = figure(...
%     'Visible','On'...
%     ,'Color',[1 1 1]...
%     ,'units','pixel'...
%     ,'OuterPosition',[100 100 resolution(1) resolution(2)]...
% );
hold on

step = 1;
tick_now = 1;

visu.isShowVehID = true;             
visu.isShowPriority = false;         
visu.isShowCoupling = false;          
visu.isShowWeight = false;           
visu.isShowHotkeyDescription = false;
nexttile
plotTrafficStatus(result,step,tick_now,[],visu)

% save fig
EvaluationParl.save_fig(fig,file_name)
