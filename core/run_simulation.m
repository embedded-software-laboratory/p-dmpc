function scenario = run_simulation(options)
    scenario = Scenario(options.angles);
    [init_poses, target_poses] = create_poses(scenario);
    
    depth = 3;
    trim_indices = ones(1, options.amount);

    % Make motionGraph Tupel
    trim_set = 'trim_set_3_1';
    motionGraphList = create_motion_graph_list(trim_set, options.amount);
    
    % Load costs for A*-Star
    load('situation_costs', 'situation_costs');

    % Set figure
    f = figure;
    f.WindowState = 'maximized';
    box on;
    axis([-36, 36, -36, 36]);
    pbaspect([1 1 1]);
    set(gca, 'ytick', [-3 3]);
    title("Iteration: 0, Time: 0");
    draw_destination(target_poses);
    draw_cars(init_poses);
    
    % Create log folder
    st = dbstack;
    namestr = st(1).name;
    sub_folder = './logs/' + string(namestr) + '_' + string(trim_set) + '_circle_' + string(options.amount) + '_depth_' + string(depth);
    
    if ~exist(sub_folder, 'dir')
        mkdir(sub_folder)
    end
    
    % Initialize video
    % i = 1;
    % video_name = fullfile(sub_folder,"video" + "(" + string(i) + ")");
    % while isfile(video_name+ ".avi")
    %     i = i + 1;
    %     video_name = fullfile(sub_folder,"video" + "(" + string(i) + ")");
    % end
    video_name = fullfile(sub_folder,"video");
    video = VideoWriter(video_name);
    video.FrameRate = 60;
    open(video)

    % Combine graphs
    combined_graph = CombinedGraph(motionGraphList);
    [video, search_tree] = receding_horizon(init_poses, target_poses, trim_indices, combined_graph, situation_costs, video);
    close(video);
    
    % Log workspace to subfolder 
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

