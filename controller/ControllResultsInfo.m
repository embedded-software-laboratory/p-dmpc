classdef ControllResultsInfo
    % CONTROLLRESULTSINFO Summary of this class goes here
    %   Detailed explanation goes here

    properties
        tree                            % object of the class `Tree`
        tree_path                       % tree path
        subcontroller_runtime           % sub-controller running time of each vehicle
        subcontroller_runtime_all_grps  % sub-controller running time of all vehicle groups
        n_expanded                      % sum of number of times that nodes are expended during graph searching of all vehicles
        next_node                       % next node information
        shapes                          % predicted occupied areas of all prediction horizons
        vehicle_fullres_path            % predicted trajectory of the next time step
        predicted_trims                 % predicted trims of all prediction horizon (including the current trim)
        y_predicted                     % predicted trajectory
        computation_levels              % actual number of computation levels of the whole system
        vehs_fallback                   % vehicles that need to take fallback
        is_exhausted                    % whether graph search is exhausted
        u                               % control input
    end

    properties (Dependent)
        trim_indices                    % predicted trim of the next time step
        subcontroller_run_time_total    % sub-controller running time of the whole system
    end

    properties (SetAccess=private)
        controller_ID                   % controller ID which should be the same as the corresponding vehicle ID
    end
    

    methods
        function obj = ControllResultsInfo(nVeh, Hp, controller_ID)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            if nargin == 2
                obj.controller_ID = 1:nVeh; % default controller ID
                warning('Default controller ID is used.')
            else
                obj.controller_ID = controller_ID;
            end
            assert(length(obj.controller_ID)==nVeh)

            obj.tree = cell(nVeh,1);
            obj.tree_path = zeros(nVeh,Hp+1);
            obj.subcontroller_runtime = zeros(nVeh,1); 
            obj.n_expanded = 0;
            obj.next_node = node(-1, zeros(nVeh,1), zeros(nVeh,1), zeros(nVeh,1), zeros(nVeh,1), -1, -1);
            obj.shapes = cell(nVeh,Hp);                   
            obj.vehicle_fullres_path = cell(nVeh,1);
            obj.predicted_trims = zeros(nVeh,Hp+1);
            obj.y_predicted = cell(nVeh,1);
            obj.computation_levels = [];
            obj.vehs_fallback = [];
            obj.is_exhausted = false(nVeh,1);
            obj.u = zeros(nVeh,1); 
        end

        function trim_indices = get.trim_indices(obj)
            % Predicted trim of the next time step
            trim_indices = obj.predicted_trims(:,2);
        end

        function subcontroller_run_time_total = get.subcontroller_run_time_total(obj)
            % the subcontroller time of the whole system in each time step
            % depends on the maximum subcontroller time used by each
            % parallel group  
            subcontroller_run_time_total = max(obj.subcontroller_runtime_all_grps);
        end

        function obj = store_control_info(obj, info_v, scenario)
            % Store the control information, such as `tree`, `tree_path`,
            % `n_expanded`, `next_node`, `shapes`, `vehicle_fullres_path`, `predicted_trims`, `y_predicted`
            vehicle_idx = find(obj.controller_ID==info_v.controller_ID);
            obj.tree{vehicle_idx} = info_v.tree;
            obj.tree_path(vehicle_idx,:) = info_v.tree_path;
            obj.n_expanded = obj.n_expanded + info_v.tree.size();
            obj.next_node = set_node(obj.next_node, vehicle_idx, info_v);
            obj.shapes(vehicle_idx,:) = info_v.shapes(:);
            obj.vehicle_fullres_path(vehicle_idx) = path_between(info_v.tree_path(1), info_v.tree_path(2), info_v.tree, scenario);
            obj.predicted_trims(vehicle_idx,:) = info_v.predicted_trims; % store the planned trims in the future Hp time steps
%             obj.trim_indices(vehicle_idx) = info_v.trim_indices; % dependent variable
            obj.y_predicted(vehicle_idx) = info_v.y_predicted; % store the information of the predicted output
        end

        function obj = get_run_time_total_all_grps(obj, parl_groups_info, CL_based_hierarchy)
        % Calculate the total runtime: in each parallel group, only one vehicle in each computation
        % level will be counted, this is the one with the maximum runtime 
        
            n_grps = length(parl_groups_info); % number of parallel groups
        
            run_time_total_all_grps = zeros(1,n_grps); % subcontroller time of each group
        
            for grp_i = 1:n_grps
                vehs_in_grp_i = parl_groups_info(grp_i).vertices;
                for level_j = 1:length(CL_based_hierarchy)
                    vehs_in_level_j = CL_based_hierarchy(level_j).members;
                    find_in_same_level = ismember(vehs_in_grp_i,vehs_in_level_j);
                    vehs_in_same_level = vehs_in_grp_i(find_in_same_level);
        
                    % take the maximum runtime among vehicles in the same group and in the same computation level
                    if ~isempty(vehs_in_same_level)
                        run_time_total_all_grps(grp_i) = run_time_total_all_grps(grp_i) + max(obj.subcontroller_runtime(vehs_in_same_level));
                    end
                end
            end
    
            obj.subcontroller_runtime_all_grps = run_time_total_all_grps;
            obj.computation_levels = length(CL_based_hierarchy);
        
        end
    end
end


