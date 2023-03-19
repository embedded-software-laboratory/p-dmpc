classdef ControllResultsInfo
    % CONTROLLRESULTSINFO Summary of this class goes here
    %   Detailed explanation goes here

    properties
        tree                            % object of the class `Tree`
        tree_path                       % tree path
        runtime_subcontroller_max       % subcontroller runtime, equals to the maximum one among all groups
        runtime_subcontroller_each_veh  % subcontroller runtime of each vehicle
        runtime_subcontroller_each_grp  % subcontroller runtime of each group
        runtime_graph_search_each_veh   % graph search time of each vehicle
        runtime_graph_search_each_grp   % graph search time of each group
        runtime_graph_search_max        % graph search time, equals to the maximum one among all groups
        n_expanded                      % number of times that nodes are expended during graph search
        next_node                       % next node information
        shapes                          % predicted occupied areas of all prediction horizons
        vehicle_fullres_path            % predicted trajectory of the next time step
        predicted_trims                 % predicted trims of all prediction horizon (including the current trim)
        trim_indices                    % predicted trim of the next time step
        y_predicted                     % predicted trajectory
        computation_levels              % actual number of computation levels of the whole system
        vehs_fallback                   % vehicles that need to take fallback
        is_exhausted                    % whether graph search is exhausted
        is_semi_exhausted               % vehicle at a stillstand but the graph search is still exhausted
        u                               % control input
    end

    properties (Dependent)
        
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
            obj.runtime_subcontroller_max = zeros(nVeh,1); 
            obj.n_expanded = zeros(nVeh,1);
            obj.next_node = node(-1, zeros(nVeh,1), zeros(nVeh,1), zeros(nVeh,1), zeros(nVeh,1), -1, -1);
            obj.shapes = cell(nVeh,Hp);                   
            obj.vehicle_fullres_path = cell(nVeh,1);
            obj.predicted_trims = zeros(nVeh,Hp+1);
            obj.y_predicted = cell(nVeh,1);
            obj.computation_levels = inf;
            obj.vehs_fallback = int32.empty;
            obj.is_exhausted = false(nVeh,1);
            obj.u = zeros(nVeh,1);
            %obj.runtime_graph_search_each_veh = zeros(nVeh);
            %obj.runtime_subcontroller_each_veh = zeros(nVeh);
        end

        function obj = store_control_info(obj, info_v, scenario)
            % Store the control information, such as `tree`, `tree_path`,
            % `n_expanded`, `next_node`, `shapes`, `vehicle_fullres_path`, `predicted_trims`, `y_predicted`
            if scenario.options.isPB
                vehicle_idx = find(obj.controller_ID==info_v.controller_ID);
                obj.tree{vehicle_idx} = info_v.tree;
                obj.tree_path(vehicle_idx,:) = info_v.tree_path;
                obj.n_expanded(vehicle_idx,1) = info_v.tree.size();
                obj.next_node = set_node(obj.next_node, vehicle_idx, info_v);
                obj.shapes(vehicle_idx,:) = info_v.shapes(:);
                obj.vehicle_fullres_path(vehicle_idx) = path_between(info_v.tree_path(1), info_v.tree_path(2), info_v.tree, scenario);
                obj.predicted_trims(vehicle_idx,:) = info_v.predicted_trims; % store the planned trims in the future Hp time steps
    %             obj.trim_indices(vehicle_idx) = info_v.trim_indices; % dependent variable
                obj.y_predicted(vehicle_idx) = info_v.y_predicted; % store the information of the predicted output
            else
                % for centralized control
                if scenario.options.use_cpp % this is stupid and I don't like it, but it is a minimally invasive procedure
                    obj.next_node = info_v.next_node;
                    obj.predicted_trims = info_v.predicted_trims;
                    obj.y_predicted = info_v.y_predicted(:);
                else
                    obj.tree = info_v.tree; % only for node explorationslee
                    obj.n_expanded = info_v.tree.size();
                    obj.next_node = set_node(obj.next_node,1:scenario.options.amount,info_v);
                    obj.shapes = info_v.shapes;
                    obj.vehicle_fullres_path = path_between(info_v.tree_path(1), info_v.tree_path(2), info_v.tree, scenario)';
                    obj.predicted_trims = info_v.predicted_trims; % store the planned trims in the future Hp time steps
%                     obj.trim_indices = info_v.trim_indices; % dependent variable
                    obj.y_predicted = info_v.y_predicted(:); % store the information of the predicted output
                end
            end

            % Predicted trim of the next time step
            obj.trim_indices = obj.predicted_trims(:,2);
        end

        function obj = get_run_time_total_all_grps(obj, parl_groups_info, CL_based_hierarchy, ...
                msg_send_time, runtime_others, runtime_planning)
        % Calculate the total runtime: in each parallel group, only one vehicle in each computation
        % level will be counted, this is the one with the maximum runtime 
            n_grps = length(parl_groups_info); % number of parallel groups
        
            obj.runtime_graph_search_each_grp = zeros(1,n_grps); % subcontroller time of each group
            obj.runtime_subcontroller_each_grp = zeros(1,n_grps);

            obj.runtime_subcontroller_each_veh = msg_send_time + runtime_planning;
        
            for grp_i = 1:n_grps
                vehs_in_grp_i = parl_groups_info(grp_i).vertices;
                for level_j = 1:length(CL_based_hierarchy)
                    vehs_in_level_j = CL_based_hierarchy(level_j).members;
                    find_in_same_level = ismember(vehs_in_grp_i,vehs_in_level_j);
                    vehs_in_same_level = vehs_in_grp_i(find_in_same_level);
        
                    % take the maximum runtime among vehicles in the same group and in the same computation level
                    if ~isempty(vehs_in_same_level)
%                         run_time_total_all_grps(grp_i) = run_time_total_all_grps(grp_i) + max(obj.subcontroller_runtime(vehs_in_same_level));
                        obj.runtime_graph_search_each_grp(grp_i) = obj.runtime_graph_search_each_grp(grp_i) + max(obj.runtime_graph_search_each_veh(vehs_in_same_level));
                        obj.runtime_subcontroller_each_grp(grp_i) = obj.runtime_subcontroller_each_grp(grp_i) + max(obj.runtime_subcontroller_each_veh(vehs_in_same_level));
                    end
                end
            end

            % graph search time depends on the maximum graph search time of all groups
            obj.runtime_subcontroller_each_veh = obj.runtime_subcontroller_each_veh + runtime_others;
            obj.runtime_subcontroller_each_grp = obj.runtime_subcontroller_each_grp + runtime_others;
            obj.runtime_graph_search_max = max(obj.runtime_graph_search_each_grp);
            obj.runtime_subcontroller_max = max(obj.runtime_subcontroller_each_grp);
            obj.computation_levels = length(CL_based_hierarchy);
        
        end
    end
end


