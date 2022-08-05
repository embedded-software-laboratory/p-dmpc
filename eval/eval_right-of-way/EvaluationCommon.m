classdef EvaluationCommon
    %EVALUATIONCOMMON Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        nVeh        % number of vehicles
        nSteps      % number of time steps
        dt          % sample time
        t_total     % total running time
        results_full_path   % path where the results are stored
        
        path_tracking_errors_MAE % mean absolute error
        path_tracking_error_average % average path tracking error

        nodes_expanded_per_step % expanded nodes when graph searching
        nodes_expanded_average % expanded nodes average over steps
        
        runtime_iter_per_step % calculate reachable sets, pridicted lanelets and reference trajectory
        runtime_subcontroller_per_step % two big parts: graph search, priority assignment
        runtime_total_per_step % total runtime per step
        runtime_average % average runtime
        runtime_max % maximum top n runtime
        
        average_speed_each_veh % average speed of each vehicle per step
        average_speed % average speed of all vehicles per step

        fallback_times % total fallback times of all vehicles. Note that this is not the number of time steps when fallbacks occur
        fallback_rate % fallback rate: fallback_times/nSteps/nVeh
    end

    properties (Access=private)
        reference_paths     % reference path of each vehicle
        real_paths          % real path of each vehicle
        path_tracking_errors % path tracking errors

        plot_option_real_path
        plot_option_ref_path
    end

    properties (Dependent)
        mean_tracking_error_per_veh
    end
    
    methods
        function obj = EvaluationCommon(results_full_path)
            obj.results_full_path = results_full_path;
            load(obj.results_full_path,'result');
            obj.nVeh = result.scenario.nVeh;
            obj.nSteps = length(result.iteration_structs);
            obj.dt = result.scenario.dt;
            obj.t_total = obj.nSteps*obj.dt;
            obj.reference_paths = cell(obj.nVeh,1);
            obj.real_paths = cell(obj.nVeh,1);
            obj.path_tracking_errors = cell(obj.nVeh,1);
            obj.path_tracking_errors_MAE = zeros(obj.nVeh,1);
            obj.runtime_total_per_step = zeros(0,1);
            obj.plot_option_real_path = struct('Color','b','LineStyle','-','LineWidth',1.0,'DisplayName','Real Path');
            obj.plot_option_ref_path = struct('Color','r','LineStyle','--','LineWidth',1.0,'DisplayName','Reference Path');

            obj = obj.get_path_tracking_errors(result);
            obj = obj.get_runtime_per_step(result);
            obj = obj.get_average_speeds;

            obj.fallback_times = 0;
            for i = 1:length(result.vehs_fallback)
                if ~isempty(result.vehs_fallback{i})
                    obj.fallback_times = obj.fallback_times + length(result.vehs_fallback{i});
                end
            end
            obj.fallback_rate = obj.fallback_times/obj.nSteps/obj.nVeh;
        end
        
        function obj = get_path_tracking_errors(obj,result)
            % Calculate the path tracking error
            for iVeh = 1:obj.nVeh
                for iIter = 1:obj.nSteps
                    obj.reference_paths{iVeh}(:,iIter) = reshape(result.iteration_structs{iIter}.referenceTrajectoryPoints(iVeh,1,:),2,[]);
                    obj.real_paths{iVeh}(:,iIter) = result.vehicle_path_fullres{iVeh,iIter}(end,1:2)';
                    obj.path_tracking_errors{iVeh}(:,iIter) = sqrt(sum((obj.real_paths{iVeh}(:,iIter) - obj.reference_paths{iVeh}(:,iIter)).^2,1));
                end
                obj.path_tracking_errors_MAE(iVeh) = mean(obj.path_tracking_errors{iVeh});
            end
            obj.path_tracking_error_average = mean(obj.path_tracking_errors_MAE);
            
%             obj.visualize_path(result)
        end

        function obj = get_runtime_per_step(obj,result)
            % Calculate the total runtime per step
            assert( abs(length(result.subcontroller_run_time_total)-length(result.iter_runtime)) <= 1 )
            obj.runtime_iter_per_step = result.iter_runtime(1:obj.nSteps)';
            obj.runtime_subcontroller_per_step = result.subcontroller_run_time_total(1:obj.nSteps)';
            obj.runtime_total_per_step = obj.runtime_iter_per_step + obj.runtime_subcontroller_per_step;

            % Find outliers
            outliers = find(isoutlier(obj.runtime_total_per_step));
            disp(['Find ' num2str(length(outliers)) ' outliers.'])
%             % set outliers to average values
%             obj.runtime_total_per_step(outliers) = obj.runtime_average;
            % ignore the first several steps as they may have higher values
            % in computation time due to the just-in-time (JIT) compilation
            if obj.nSteps > 20
                steps_ignored = 3;
            else
                steps_ignored = 1;
            end
            obj.runtime_average = sum(obj.runtime_total_per_step(steps_ignored+1:end))/(obj.nSteps-steps_ignored);
            obj.runtime_max = maxk(obj.runtime_total_per_step(steps_ignored+1:end),10); 

            obj.nodes_expanded_per_step = result.n_expanded;
            obj.nodes_expanded_average = mean(obj.nodes_expanded_per_step);
            
        end

        function obj = get_average_speeds(obj)
            % Calculates the average speed of each vehicle
            % note that we use reference path but not real path
            ref_paths_diff = cellfun(@(c) diff(c,1,2),obj.reference_paths,'UniformOutput',false);
            distances = cellfun(@(c) sum(sqrt(c(1,:).^2+c(2,:).^2)),ref_paths_diff);
            obj.average_speed_each_veh = distances./obj.t_total;
            obj.average_speed = mean(obj.average_speed_each_veh);
        end


        function visualize_path(obj,result)
            % visualize reference path and real path
            figure() 
            hold on
            box on
            axis equal
            
            xlabel('\fontsize{14}{0}$x$ [m]','Interpreter','LaTex');
            ylabel('\fontsize{14}{0}$y$ [m]','Interpreter','LaTex');
        
            xlim(result.scenario.plot_limits(1,:));
            ylim(result.scenario.plot_limits(2,:));
            daspect([1 1 1])

            plot_lanelets(result.scenario.lanelets,result.scenario.name)

            % get vehicle that has maximum path tracking error
            [~,vehs_max_error] = maxk(obj.path_tracking_errors_MAE,5);
            % get vehicle that has minimum path tracking error
            [~,vehs_min_error] = mink(obj.path_tracking_errors_MAE,5); 

           veh_max_error = vehs_max_error(1);
           veh_min_error = vehs_min_error(1);
            s_max = obj.plot_path(obj.real_paths{veh_max_error},obj.path_tracking_errors{veh_max_error});
            
            c = colorbar;
            c.Title.String = 'Path Tracking Error';
            p_max = plot(obj.reference_paths{veh_max_error}(1,:),obj.reference_paths{veh_max_error}(2,:),obj.plot_option_ref_path);
            legend([s_max,p_max],{'Real Path (max error)','Reference Path (max error)'})
%             s_min = obj.plot_path(obj.real_paths{veh_min_error},obj.path_tracking_errors{veh_min_error});
%             p_min = plot(obj.reference_paths{veh_min_error}(1,:),obj.reference_paths{veh_min_error}(2,:),obj.plot_option_ref_path);
%             legend([s_max,s_min,p_max,p_min],{'Real Path (max error)','Real Path (min error)','Reference Path (max error)','Reference Path (min error)'})
        end

        function s = plot_path(~,path,error)
            x = path(1,:); y = path(2,:);
            z = zeros(size(x));
            lineColor = error;
            s = surface([x;x], [y;y], [z;z], [lineColor;lineColor],'FaceColor', 'no','EdgeColor', 'interp','LineWidth', 4);
        end
        
    end

end

