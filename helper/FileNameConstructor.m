classdef FileNameConstructor
    % FILENAMECONSTRUCTOR This class provides methods to construct
    % different file names according to certain rules. 

    properties

    end
 

    methods
        function obj = FileNameConstructor()
        end
    end

    methods
 
    end

    methods (Static)
        function mpa_instance_name = get_mpa_name(options)
            % GET_MPA_NAME Construct name for the file to which a object of the
            % class MPA is saved.
            % Example:  MPA_trims12_Hp6
            %           MPA_trims12_Hp6_parl_non-convex

            mpa_instance_name = ['MPA_','trims',num2str(options.trim_set),'_Hp',num2str(options.Hp),'_T',num2str(options.dt)];

            if options.is_allow_non_convex
                mpa_instance_name = [mpa_instance_name,'_non-convex'];
            end

            if ~options.isPB
                mpa_instance_name = [mpa_instance_name,'_centralized_nVeh' num2str(options.amount)];
            end

            if options.is_use_dynamic_programming
                mpa_instance_name = [mpa_instance_name,'_DP'];
            end
            
            if ~options.recursive_feasibility
                mpa_instance_name = [mpa_instance_name,'_notRecFeas'];
            end

            mpa_instance_name = [mpa_instance_name,'.mat'];
        end

        function results_folder_path = gen_results_folder_path(options)
            
            if options.isPB
                controller_name = 'par-rhgs';
            else
                controller_name = 'cen-rhgs';
            end
            
            priority = char(options.priority);            

            results_folder_name = strrep(strcat(char(options.scenario_name), '_', controller_name),' ','_');

            [file_path,~,~] = fileparts(mfilename('fullpath')); % get the path of the current file
            idcs = strfind(file_path,filesep); % find all positions of '/'
            main_folder = file_path(1:idcs(end)-1); % one folder up
        
            results_folder_path = fullfile(main_folder,'results',results_folder_name);
            if ~isfolder(results_folder_path)
                % create target folder if not exist
                mkdir(results_folder_path)
            end
            
        end

        function video_file_path = gen_video_file_path(options)
            video_file_name = [...
                'video_', ...
                FileNameConstructor.gen_scenario_name(options,options.amount), ...
                '.avi' ...
            ];
            video_file_path = fullfile( ...
                FileNameConstructor.gen_results_folder_path(options) ...
                ,video_file_name...
            );
        end

        function scenario_name = gen_scenario_name(options, vehs)
            priority = char(options.priority);


            if isempty(options.customResultName)
                % use default name
                if options.isParl
                    scenario_name = ['veh_', num2str(options.veh_ids(vehs)),'_trims',num2str(options.trim_set),'_Hp',num2str(options.Hp),'_dt',num2str(options.dt),'_nVeh',num2str(options.amount),'_T',num2str(options.T_end),'_',priority];
                else
                    scenario_name = ['trims',num2str(options.trim_set),'_Hp',num2str(options.Hp),'_dt',num2str(options.dt),'_nVeh',num2str(options.amount),'_T',num2str(options.T_end),'_',priority];
                end
                veh_ids_str = sprintf('-%d',options.veh_ids);
                scenario_name = [scenario_name, '_ids', veh_ids_str];
                if options.isPB
                    scenario_name = [scenario_name,'_maxCLs',num2str(options.max_num_CLs),...
                        '_ConsiderVehWithoutROW',options.strategy_consider_veh_without_ROW,'_EnterLaneletCrossingArea',options.strategy_enter_lanelet_crossing_area];                 
                end

                if options.isAllowInheritROW
                    scenario_name = [scenario_name,'_inherit'];
                end

                if options.is_free_flow
                    scenario_name = [scenario_name,'_freeFlow'];
                end

                if ~strcmp(options.fallback_type,'localFallback')
                    % local fallback is the default fallback strategy
                    scenario_name = [scenario_name,'_',options.fallback_type];
                end

                if ~options.isSaveResultReduced
                    scenario_name = [scenario_name,'_fullResult'];
                end

                if ~isempty(options.random_idx) && options.random_idx~=1
                    scenario_name = [scenario_name,'_random',num2str(options.random_idx)];
                end

                if ~options.isDealPredictionInconsistency
                    scenario_name = [scenario_name,'_notDealWithPredictionInconsistency'];
                end
                
                if ~strcmp(options.coupling_weight_mode,'STAC')
                    scenario_name = [scenario_name,'_W',options.coupling_weight_mode];
                end

                if ~options.bound_reachable_sets
                    scenario_name = [scenario_name,'_unboundedRS'];
                end
            else
                % use custom name 
                scenario_name = options.customResultName;
            end
        end

        function results_full_path = get_results_full_path(options, vehs)
            % GET_RESULTS_FULL_PATH Construct name for the folder where simulation
            % results are saved.
            % INPUT: options, vehs(vehicles for which this HLC is responsible.)
            results_name = [FileNameConstructor.gen_scenario_name(options, vehs), '.mat'];

            results_full_path = fullfile( ...
                FileNameConstructor.gen_results_folder_path(options) ...
                ,results_name ...
            );
        end
    end
end