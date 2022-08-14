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
        function mpa_instance_name = get_mpa_name(trim_ID,Hp,dt,isParl,is_allow_non_convex)
            % GET_MPA_NAME Construct name for the file to which a object of the
            % class MPA is saved.
            % Example:  MPA_trims12_Hp6
            %           MPA_trims12_Hp6_parl_non-convex

            mpa_instance_name = ['MPA_','trims',num2str(trim_ID),'_Hp',num2str(Hp),'_T',num2str(dt)];

            if isParl
                mpa_instance_name = [mpa_instance_name,'_parl'];                
            end

            if is_allow_non_convex
                mpa_instance_name = [mpa_instance_name,'_non-convex'];
            end

            mpa_instance_name = [mpa_instance_name,'.mat'];
        end

        function results_full_path = get_results_full_path(options)
            % GET_RESULTS_FULL_PATH Construct name for the folder where simulation
            % results are saved.
            if options.isParl
                controller_name = 'RHC-Parl';
            else
                controller_name = 'RHC';
            end

            if isstring(options.priority)
                options.priority = char(options.priority);
            end

            if isempty(options.customResultName)
                % use default name
                results_name = ['trims',num2str(options.trim_set),'_Hp',num2str(options.Hp),'_dt',num2str(options.dt),'_nVeh',num2str(options.amount),'_T',num2str(options.T_end),'_',options.priority];
    
                if options.isParl
                    results_name = [results_name,'_maxCLs',num2str(options.max_num_CLs),...
                        '_ConsiderVehWithoutROW',options.strategy_consider_veh_without_ROW,'_EnterLaneletCrossingArea',options.strategy_enter_lanelet_crossing_area];                 
                end

                if options.isAllowInheritROW
                    results_name = [results_name,'_inherit'];
                end

                if options.is_free_flow
                    results_name = [results_name,'_freeFlow'];
                end

                if ~strcmp(options.fallback_type,'localFallback')
                    % local fallback is the default fallback strategy
                    results_name = [results_name,'_',options.fallback_type];
                end

                if options.isSaveResultReduced
                    results_name = [results_name,'_reduced'];
                    
                end
            else
                % use custom name 
                results_name = options.customResultName;
            end

            results_folder = strrep(strcat(options.scenario, '_', controller_name),' ','_');
            results_name = [results_name, '.mat'];

            [file_path,~,~] = fileparts(mfilename('fullpath')); % get the path of the current file
            idcs = strfind(file_path,filesep); % find all positions of '/'
            one_folder_up = file_path(1:idcs(end)-1); % one folder up
        
            folder_target = fullfile(one_folder_up,'results',results_folder);
            if ~isfolder(folder_target)
                % create target folder if not exist
                mkdir(folder_target)
            end        
            results_full_path = fullfile(folder_target,results_name);
        end
    end
end