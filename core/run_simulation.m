function scenario = run_simulation(options)
    scenario = Scenario(options.angles);
    [init_poses, target_poses] = create_poses(scenario);
    
    trim_indices = ones(1, options.amount);

    % Make motionGraph Tupel
    trim_set = 'trim_set_3_1';
    motionGraphList = create_motion_graph_list(trim_set, options.amount);
    
    % Load costs for A*-Star
    load('situation_costs', 'situation_costs');

    % Set figure
    f = figure;
    f.WindowState = 'maximized';
    set(f,'color','w');
    set(gca, 'FontSize', 35);
    set(gca,'TickLabelInterpreter','latex');
    set(gca,'Box','on')
    xlabel('$x$ [m]', 'Interpreter','Latex')
    ylabel('$y$ [m]', 'Interpreter','Latex')
    box on;
    axis([-18, 18, -18, 18]);
    %set(gca, 'ytick', [-9 9]);
    pbaspect([1 1 1]);
    title("Iteration: 0, Time: 0");
    draw_destination(target_poses);
    draw_cars(init_poses);
    obstacles = [];
    %obstacles = polyshape([-2 -2 2 2],[4 -4 -4 4]);
    %plot(obstacles, 'EdgeColor', 'blue', 'LineWidth', 2);
    
    % Create log folder
    st = dbstack;
    namestr = st(1).name;
    sub_folder = './logs/' + string(namestr) + '_' + string(trim_set) + '_circle_' + string(options.amount) + '_' + string(h_u) + '_' + string(h_p);
    
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
    video.FrameRate = 1;
    open(video)

    % Combine graphs
    combined_graph = CombinedGraph(motionGraphList);
    [video, search_tree, n_vertices] = receding_horizon(init_poses, target_poses, trim_indices, obstacles, combined_graph, situation_costs, video);
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

