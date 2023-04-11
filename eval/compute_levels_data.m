function [ data ] = compute_levels_data(res)
% COMPUTE_LEVELS_DATA  Calculate data needed to export plots for compuation level evaluation
    
    data = struct();

    resstruct = [res{:}];
    runtimes = [resstruct.t_total];
    is_equal_runtime = all(runtimes == runtimes(1));
    if ~is_equal_runtime
        warning("different experiment durations");
    end

    [ nVeh, nPri, nSce ] = size(res);
    nLevels_by_veh_pri = cell(nVeh, nPri);
    nLevels_by_pri = cell(nPri,1);
    nVeh_list = zeros(1,nVeh);
    for iVeh = 1:nVeh
        nVeh_list(iVeh) = res{iVeh,1,1}.scenario.options.amount;
        for iPri = 1:nPri
            for iSce = 1:nSce
                result = res{iVeh,iPri,iSce};
        
                % get number of steps until deadlock
                [nSteps,~] = compute_deadlock_free_runtime(result);

                if ~(nSteps == result.nSteps)
                    continue;
                end
        
                scenario_tmp = result.scenario;
        
                for iStep = nSteps:-1:1
                    % no adjacency given in 1-veh scenarios
                    if scenario_tmp.options.amount > 1
                        scenario_tmp.adjacency = scenario_tmp.adjacency(:,:,1:iStep);
            
                        iter_tmp = result.iteration_structs{iStep};
                    
                        % assign priorities using different algorithms
                        [~, ~, ~, fca_prios] = FcaPriority().priority(scenario_tmp,iter_tmp);
                        [~, ~, random_prios] = RandomPriority().priority(scenario_tmp);
                        [~, ~, constant_prios] = ConstantPriority().priority(scenario_tmp);
                        [~, ~, coloring_prios] = ColoringPriority().priority(scenario_tmp);
            
                        % get number of levels by max priority assigned
                        n_fca = max(fca_prios);
                        n_random = max(random_prios);
                        n_constant = max(constant_prios);
                        n_coloring = max(coloring_prios);
                        disp(['Scenario ',num2str(iVeh),'/',num2str(iPri),'/',num2str(iSce),' Step ',num2str(nSteps-iStep),' of ',num2str(nSteps)]);
                    else
                        [n_fca, n_random, n_constant, n_coloring] = deal(1);
                    end

                    
                    nLevels_by_veh_pri{iVeh,1} = [nLevels_by_veh_pri{iVeh,1} n_fca];
                    nLevels_by_veh_pri{iVeh,2} = [nLevels_by_veh_pri{iVeh,2} n_random];
                    nLevels_by_veh_pri{iVeh,3} = [nLevels_by_veh_pri{iVeh,3} n_constant];
                    nLevels_by_veh_pri{iVeh,4} = [nLevels_by_veh_pri{iVeh,4} n_coloring];
        
                    nLevels_by_pri{1} = [nLevels_by_pri{1} n_fca];
                    nLevels_by_pri{2} = [nLevels_by_pri{2} n_random];
                    nLevels_by_pri{3} = [nLevels_by_pri{3} n_constant];
                    nLevels_by_pri{4} = [nLevels_by_pri{4} n_coloring];
                end
            end
        end
    end

    data.nLevels_by_veh_pri = nLevels_by_veh_pri;
    data.nLevels_by_pri = nLevels_by_pri;
    data.nVeh_list = nVeh_list;
    data.result = res{1,1,1};
    data.nVeh = nVeh;
    data.nPri = nPri;
    data.nSce = nSce;
end

