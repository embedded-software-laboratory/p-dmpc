classdef FreeFlowSpeed
    %FREEFLOWSPEED Calculate free-flow speed of different sample times
    % Free-flow speed: free movement, not constrained by other vehicles

    properties
        sample_time = [0.2,0.3,0.4];
        free_flow_speed = [];
    end

    properties (Access = private)
        full_path % full path of the offline data
    end

    methods
        function obj = FreeFlowSpeed(scenario)

            [file_path,~,~] = fileparts(mfilename('fullpath'));
            obj.full_path = fullfile(file_path,'freeFlowSpeed.mat');

            if isfile(obj.full_path)
                disp('Offline data was found and loaded.')
                load(obj.full_path,"obj")
                return
            end

            dt_s = obj.sample_time; % different sample times
            evaluations = cell(length(dt_s),1);
            
            % prepare simulation options
            options = OptionsMain;
            options.environment = Environment.Simulation;
            options.customResultName = '';
            options.scenario_name = 'Commonroad';
            options.trim_set = 9;
            options.Hp = 5;
            options.priority = 'STAC_priority';
            options.T_end = 20;
            options.isPB = true;
            options.isAllowInheritROW = true;
            options.max_num_CLs = 2;
            options.strategy_consider_veh_without_ROW = '3';
            options.strategy_enter_lanelet_crossing_area = '4';
            options.isSaveResult = true;
            options.visu = [false,false];
            options.is_eval = false;
            options.visualize_reachable_set = false;
            options.amount = 20;
            options.is_free_flow = true;
            
            for i = 1:length(dt_s)
                options.dt = dt_s(i);
            
                results_full_path = FileNameConstructor.get_results_full_path(options);
                if isfile(results_full_path)
                    disp('File already exists.')
                else
                    % run simulation
                    if exist('options','var') && exist('scenario','var')
                        [~,~,~] = main(options,scenario);
                    else
                        [~,scenario,~] = main(options);
                    end
                    disp('Pause to cool the CPU...')
                    pause(3) 
                end
            
                % evaluate
                evaluations{i} = EvaluationParl(results_full_path);
            
                disp([num2str(i) ': done.'])
            end
            disp('Finished.')
            
            
            for i = 1:length(dt_s)
                obj.free_flow_speed(i) = evaluations{i}.average_speed;
                disp(['Free-flow speed for sample time ' num2str(dt_s(i)) ' seconds: ' num2str(evaluations{i}.average_speed) ' [m/s].'])
            end

            % save data
            save(obj.full_path,'obj')
            disp(['Data was saved under ' obj.full_path])
        end

    end
end