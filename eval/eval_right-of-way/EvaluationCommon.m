classdef EvaluationCommon
    %EVALUATIONCOMMON Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        result
        nVeh
        nSteps
        results_full_path
        reference_paths
        real_paths
        path_tracking_errors
        path_tracking_errors_MAE % mean absolute error 
        total_runtime_per_step
        plot_option_real_path
        plot_option_ref_path

    end

    properties (Dependent)
        mean_tracking_error_per_veh
    end
    
    methods
        function obj = EvaluationCommon(results_full_path)
            obj.results_full_path = results_full_path;
            load(obj.results_full_path,'result')
            disp(result)
            obj.result = result;
            obj.nVeh = result.scenario.nVeh;
            obj.nSteps = length(result.iteration_structs);
            obj.reference_paths = cell(obj.nVeh,1);
            obj.real_paths = cell(obj.nVeh,1);
            obj.path_tracking_errors = cell(obj.nVeh,1);
            obj.path_tracking_errors_MAE = zeros(obj.nVeh,1);
            obj.total_runtime_per_step = zeros(0,1);
            obj.plot_option_real_path = struct('Color','b','LineStyle','-','LineWidth',1.0,'DisplayName','Real Path');
            obj.plot_option_ref_path = struct('Color','r','LineStyle','--','LineWidth',1.0,'DisplayName','Reference Path');
            
        end
        
        function obj = get_path_tracking_errors(obj)
            % Calculate the path tracking error
            for iVeh = 1:obj.nVeh
                for iIter = 1:obj.nSteps
                    obj.reference_paths{iVeh}(:,iIter) = reshape(obj.result.iteration_structs{iIter}.referenceTrajectoryPoints(iVeh,1,:),2,[]);
                    obj.real_paths{iVeh}(:,iIter) = obj.result.vehicle_path_fullres{iVeh,iIter}(end,1:2)';
                    obj.path_tracking_errors{iVeh}(:,iIter) = sqrt(sum((obj.real_paths{iVeh}(:,iIter) - obj.reference_paths{iVeh}(:,iIter)).^2,1));
                end
                obj.path_tracking_errors_MAE(iVeh) = mean(obj.path_tracking_errors{iVeh});
            end
            
            obj.visualize_path()

        end

        function obj = get_total_runtime_per_step(obj)
            % Calculate the total runtime per step
            assert( abs(length(obj.result.subcontroller_run_time_total)-length(obj.result.iter_runtime)) <= 1 )
            obj.total_runtime_per_step = obj.result.subcontroller_run_time_total(1:obj.nSteps)' + obj.result.iter_runtime(1:obj.nSteps)';
        end

        function visualize_path(obj)
            % visualize reference path and real path
            figure() 
            hold on
            box on
            axis equal
            
            xlabel('\fontsize{14}{0}$x$ [m]','Interpreter','LaTex');
            ylabel('\fontsize{14}{0}$y$ [m]','Interpreter','LaTex');
        
            xlim(obj.result.scenario.plot_limits(1,:));
            ylim(obj.result.scenario.plot_limits(2,:));
            daspect([1 1 1])

            plot_lanelets(obj.result.scenario.lanelets,obj.result.scenario.name)

            % get vehicle that has maximum path tracking error
            [~,vehs_max_error] = maxk(obj.path_tracking_errors_MAE,5);
            % get vehicle that has minimum path tracking error
            [~,vehs_min_error] = mink(obj.path_tracking_errors_MAE,5); 

           veh_max_error = vehs_max_error(1);
           veh_min_error = vehs_min_error(1);
            s = obj.plot_path(obj.real_paths{veh_max_error},obj.path_tracking_errors{veh_max_error});
            c = colorbar;
            c.Title.String = 'Path Tracking Error';
%             obj.plot_option_real_path.DisplayName = 'Real Path with Maximum Error';
%             p1 = plot(obj.real_paths{veh_max_error}(1,:),obj.real_paths{veh_max_error}(2,:),obj.plot_option_real_path);
            p2 = plot(obj.reference_paths{veh_max_error}(1,:),obj.reference_paths{veh_max_error}(2,:),obj.plot_option_ref_path);
%             obj.plot_option_real_path.DisplayName = 'Real Path with Minimum Error';
%             p3 = plot(obj.real_paths{veh_min_error}(1,:),obj.real_paths{veh_min_error}(2,:),obj.plot_option_real_path);
%             p4 = plot(obj.reference_paths{veh_min_error}(1,:),obj.reference_paths{veh_min_error}(2,:),obj.plot_option_ref_path);
%             legend('Reference Path','Real Path')
            legend([s,p2],{'Real Path','Reference Path'})
        end

        function s = plot_path(~,path,error)
            x = path(1,:); y = path(2,:);
            z = zeros(size(x));
            lineColor = error;
            s = surface([x;x], [y;y], [z;z], [lineColor;lineColor],'FaceColor', 'no','EdgeColor', 'interp','LineWidth', 4);
        end
        
    end

end

