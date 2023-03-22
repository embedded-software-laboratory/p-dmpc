%% Evaluate the weighting function for couplings

% prepare simulation options
options = OptionsMain;
options.environment = Environment.Simulation;
options.customResultName = '';
options.scenario_name = 'Commonroad';
options.trim_set = 9;
options.Hp = 5;

options.T_end = 10;
options.dt = 0.2;
options.max_num_CLs = 2;
options.priority = 'STAC_priority';
options.isPB = true;
options.isAllowInheritROW = false;
options.isSaveResult = true;
options.isSaveResultReduced = true;
options.is_plot_online = false;
options.is_eval = false;
options.strategy_consider_veh_without_ROW = '3';
options.strategy_enter_lanelet_crossing_area = '1';
options.isSaveResultReduced = true;

options.coupling_weight_mode  = {'STAC','random','constant','optimal'};

options.amount = 20;

random_stream = RandStream('mt19937ar');
options.veh_ids = sort(randsample(random_stream,9:40,options.amount),'ascend');

full_path = FileNameConstructor.get_results_full_path(options);
if isfile(full_path)
    disp('File already exists.')
else
    % run simulation
    if exist('options','var') && exist('scenario','var')
        [~,~,~] = main(options,scenario);
    else
        [~,scenario,~] = main(options);
    end
end
load(full_path,'result')
% display progress

disp('--------Finished--------')

%% Plot

fig_x = 16;     fig_y = 8; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

visu.plot_vehicle_id = true;                % is show vehicle IDs
visu.plot_priority = false;            % is show priority colorbar
visu.plot_coupling = false;             % is show coupling edges
visu.plot_weight = false;              % is show coupling weights
visu.plot_hotkey_description = false;   % is show description of hotkeys
visu.color_mode = 'group'; % vehicles in the same group have the same color

step_index  = 35;
tick = 1;
file_name_s = {'evalWeightingFunction_optimal','evalWeightingFunction'};

for i = 1:2

    file_name = file_name_s{i};
    fig = figure('Name',file_name);
    set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
    set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
        'PaperPosition', [x_margin y_margin fig_x_position fig_y_position])

    tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
    if i==1
        coupling_weights = result.coupling_weights_optimal{step_index};
    else
        coupling_weights = result.coupling_weights_reduced{step_index};
    %     coupling_weights = coupling_weights./max(coupling_weights,[],"all");
    end

    % form parallel CL_based_hierarchy
    [~, ~, belonging_vector] = form_parallel_groups(coupling_weights, options.max_num_CLs, result.coupling_info{step_index}, 'method', 's-t-cut');
    result.belonging_vector(:,step_index) = belonging_vector;
    
    nexttile
    plotTrafficStatus(result,step_index,tick,[],visu)
    set(gca,'xticklabel',[])
    set(gca,'xtick',[])
    set(gca,'yticklabel',[])
    set(gca,'ytick',[])
    xlabel('(a) Resulting parallel groups.')

    G = digraph(coupling_weights);
    n_edges = size(G.Edges.EndNodes,1);
    LineStyle = cell(1,n_edges);
    LineWidth = zeros(1,n_edges);
    for iE=1:size(G.Edges.EndNodes,1)
        if belonging_vector(G.Edges.EndNodes(iE,1)) == belonging_vector(G.Edges.EndNodes(iE,2))
            % coupling inside group in solid line
            LineStyle{iE} = '-';
            LineWidth(iE) = 0.75;
        else
            % coupling corss group in dashed line
            LineStyle{iE} = '--';
            LineWidth(iE) = 0.25;
        end
    end
    nexttile
    p = plot(G,'LineStyle',LineStyle,'Layout','layered','MarkerSize',6,'NodeColor','k','LineWidth',LineWidth,'EdgeLabel',round(G.Edges.Weight,2),'EdgeColor','k','EdgeFontSize',9,'EdgeFontName','Times New Roman');
    p.NodeLabel = {};
    node_names = 1:options.amount;
    for j=1:length(node_names)
        text(p.XData(j)+0.09, p.YData(j)+0.12, num2str(node_names(j)),'FontName','Times New Roman');
    end
    xlabel('(b) Resulting coupling graph.')
    EvaluationParl.save_fig(fig,file_name)
end

%%
fig_x = 10;     fig_y = 10; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = {'evalWeightingFunction_couplingGraph_optimal','evalWeightingFunction_couplingGraph'};
for i = 1:length(file_name)
    fig = figure('Name',file_name{i});
    set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
    set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
        'PaperPosition', [x_margin y_margin fig_x_position fig_y_position])
    
    t_fig = tiledlayout(1,1,'Padding','tight','TileSpacing','tight');
    if i==1
        coupling_weights = result.coupling_weights_optimal{step_index};
    else
        coupling_weights = result.coupling_weights_reduced{step_index};
    %     coupling_weights = coupling_weights./max(coupling_weights,[],"all");
    end
    % form parallel CL_based_hierarchy
    [~, ~, belonging_vector] = form_parallel_groups(coupling_weights, options.max_num_CLs, result.coupling_info{step_index}, 'method', 's-t-cut');
    
    % graphs_visualization(belonging_vector,coupling_weights,'ShowWeights',true)
    G = digraph(coupling_weights);
    n_edges = size(G.Edges.EndNodes,1);
    LineStyle = cell(1,n_edges);
    LineWidth = zeros(1,n_edges);
    for iE=1:size(G.Edges.EndNodes,1)
        if belonging_vector(G.Edges.EndNodes(iE,1)) == belonging_vector(G.Edges.EndNodes(iE,2))
            % coupling inside group in solid line
            LineStyle{iE} = '-';
            LineWidth(iE) = 0.75;
        else
            % coupling corss group in dashed line
            LineStyle{iE} = '--';
            LineWidth(iE) = 0.25;
        end
    end
    
    p = plot(G,'LineStyle',LineStyle,'Layout','layered','MarkerSize',6,'NodeColor','k','LineWidth',LineWidth,'EdgeLabel',round(G.Edges.Weight,2),'EdgeColor','k','EdgeFontSize',9,'EdgeFontName','Times New Roman');
    p.NodeLabel = {};
    node_names = 1:options.amount;
    for j=1:length(node_names)
        text(p.XData(j)+0.09, p.YData(j)+0.12, num2str(node_names(j)),'FontName','Times New Roman');
    end
    
    EvaluationParl.save_fig(fig,file_name{i})
end