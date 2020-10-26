function scenario = run_simulation(options)
    scenario = Scenario(options.angles);
    [init_poses, target_poses] = create_poses(scenario);
    
    depth = 3;
    trim_indices = 2 * ones(1, options.amount);

    % make motionGraph Tupel
    trim_set = 'trim_set_3_1';
    motionGraphList = create_motion_graph_list(trim_set, options.amount);

    % Set figure
    figure('units','normalized','outerposition',[0.125 0.125 0.75 0.75]);
    pbaspect([1 1 1]);
    axis([-35 35 -35 35]);
    title("Iteration: 0, Time: 0");
    draw_destination(target_poses);
    draw_cars(init_poses);
    
    % Combine graphs
    combined_graph = CombinedGraph(motionGraphList);
    search_tree = receding_horizon(init_poses, target_poses, trim_indices, combined_graph, depth, @is_collision, @get_next_node_weighted_astar);
    
    %% Log workspace to subfolder 
    st = dbstack;
    namestr = st(1).name;
    sub_folder = './logs/' + string(namestr) + '_' + trim_set + '_circle_' + string(options.amount) + '_depth_' + string(depth);
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
end

