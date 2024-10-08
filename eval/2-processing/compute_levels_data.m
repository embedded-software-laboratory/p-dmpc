function [data] = compute_levels_data(res, optional)

    arguments
        res cell
        optional.recompute (1, 1) logical = true
        optional.file_path (1, :) char = 'results/levels.mat'
    end

    % COMPUTE_LEVELS_DATA  Calculate data needed to export plots for compuation level evaluation

    if ~optional.recompute && isfile(optional.file_path)

        load(optional.file_path, 'data');
        return;

    end

    data = struct();

    resstruct = [res{:}];
    runtimes = [resstruct.t_total];
    is_equal_runtime = all(runtimes == runtimes(1));

    if ~is_equal_runtime
        warning("different experiment durations");
    end

    [nVeh, nPri, nSce] = size(res);
    nLevels_by_veh_pri = cell(nVeh, nPri);
    nLevels_by_pri = cell(nPri, 1);
    nVeh_list = zeros(1, nVeh);

    for iVeh = 1:nVeh
        nVeh_list(iVeh) = res{iVeh, 1, 1}.options.amount;

        for iPri = 1:nPri

            for iSce = 1:nSce
                experiment_result = res{iVeh, iPri, iSce};

                % get number of steps until deadlock
                [n_steps, ~] = compute_deadlock_free_runtime(experiment_result);

                if ~(n_steps == experiment_result.n_steps)
                    continue;
                end

                options_tmp = experiment_result.options;
                scenario_tmp = Scenario.create(options);

                for iStep = n_steps:-1:1
                    % no adjacency given in 1-veh scenarios
                    if options_tmp.amount > 1
                        iter_tmp = experiment_result.iteration_data(iStep);

                        % assign priorities using different algorithms
                        [~, ~, ~, fca_prios] = FcaPriority().priority(scenario_tmp, iter_tmp);
                        [~, ~, random_prios] = RandomPriority().priority(scenario_tmp, iter_tmp);
                        [~, ~, constant_prios] = ConstantPriority().priority(scenario_tmp, iter_tmp);
                        [~, ~, coloring_prios] = ColoringPriority().priority(iter_tmp);

                        % get number of levels by max priority assigned
                        n_fca = max(fca_prios);
                        n_random = max(random_prios);
                        n_constant = max(constant_prios);
                        n_coloring = max(coloring_prios);
                        disp(['Scenario ', num2str(iVeh), '/', num2str(iPri), '/', num2str(iSce), ' Step ', num2str(n_steps - iStep), ' of ', num2str(n_steps)]);
                    else
                        [n_fca, n_random, n_constant, n_coloring] = deal(1);
                    end

                    nLevels_by_veh_pri{iVeh, 1} = [nLevels_by_veh_pri{iVeh, 1} n_fca];
                    nLevels_by_veh_pri{iVeh, 2} = [nLevels_by_veh_pri{iVeh, 2} n_random];
                    nLevels_by_veh_pri{iVeh, 3} = [nLevels_by_veh_pri{iVeh, 3} n_constant];
                    nLevels_by_veh_pri{iVeh, 4} = [nLevels_by_veh_pri{iVeh, 4} n_coloring];

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
    data.result = res{1, 1, 1};
    data.nVeh = nVeh;
    data.nPri = nPri;
    data.nSce = nSce;

    save(optional.file_path, 'data');
end
