classdef  random_priority < interface_priority
% random_priority  Instance of interface_priority used for dynamic priority
% assignment, randomly assign priority to vehicles
    
    properties (Access=private)
    end
    
    methods 
        function obj = random_priority() 
            obj.is_assign_unique_priority = false; % whether to asign unique priority
        end
        
        function [parl_groups_info, CL_based_hierarchy, directed_adjacency, priority_list] = priority(obj,options)
            % This function assign, firtly, vehicles to random groups.
            % Vechile assigned earlier has a higher priority than vehicle
            % assigned to the same group later. The number of groups is
            % calculated based on the maximum number of allowed computation
            % levels. If parallel computation is not used, the maximum
            % number of computation level is set to the number of vehicles.
            nVeh = options.amount;

            if ~options.isParl
                % set the maximum number of computation levels to the number of vehicles
                % (i.e., all vehicles are in the different computation
                % levels and they plan trajectory one after another)
                options.max_num_CLs = nVeh;
            end
            
            n_grps = ceil(nVeh/options.max_num_CLs);
            parl_groups_info(n_grps) = struct('vertices',[],'num_CLs',[]);
            directed_adjacency = zeros(nVeh,nVeh);

            random_arr = randperm(nVeh,nVeh);
            % from random groups
            for i = 1:nVeh
                iVeh = random_arr(i);
                i_grp = mod(iVeh,n_grps)+1;
                current_vehs = parl_groups_info(i_grp).vertices;
                parl_groups_info(i_grp).vertices = [current_vehs,iVeh];

                % vehicle added earlier has the right-of-way over vehicle
                % added later
                if ~isempty(current_vehs)
                    directed_adjacency(current_vehs(end),iVeh) = 1;
                end
            end

            assert(nnz(directed_adjacency)==nVeh-n_grps)

            % get number of computation levels
            num_CLs_all = cellfun(@(s) {length(s)}, {parl_groups_info.vertices});
            [parl_groups_info.num_CLs] = num_CLs_all{:};

            num_CLs = max([parl_groups_info.num_CLs]);
            CL_based_hierarchy(num_CLs) = struct('members',[],'predecessors',[]); % gather vehicles that are in the same computation level 

            % determine computation level based hierarchy
            for level_i = 1:num_CLs
                for grp_i = 1:n_grps
                    if level_i <= parl_groups_info(grp_i).num_CLs
                        CL_based_hierarchy(level_i).members = [CL_based_hierarchy(level_i).members,...
                            parl_groups_info(grp_i).vertices(level_i)];
                    end
                end
                if level_i < num_CLs
                    % members of the current computation level is the
                    % predecessors of the next computation level
                    CL_based_hierarchy(level_i+1).predecessors = CL_based_hierarchy(level_i).members;
                end
            end

            % Assign prrority according to computation level
            % Vehicles with higher priorities plan trajectory before vehicles
            % with lower priorities            
            priority_list = obj.get_priority(CL_based_hierarchy);
        end
      
    end
    


end