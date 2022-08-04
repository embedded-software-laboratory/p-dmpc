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

        function results_full_path = get_results_full_path(customResultName,scenario_name,controller_name,trim_ID,...
                Hp,dt,nVeh,T_end,priority_option,isParl,max_num_CLs,strategy_consider_veh_without_ROW,strategy_enter_lanelet_crossing_area)
            % GET_RESULTS_FULL_PATH Construct name for the folder where simulation
            % results are saved.
            results_folder = strrep(strcat(scenario_name, '_', controller_name),' ','_');
            if isstring(priority_option)
                priority_option = char(priority_option);
            end

            if isempty(customResultName)
                % use default name
                results_name = ['trims',num2str(trim_ID),'_Hp',num2str(Hp),'_dt',num2str(dt),'_nVeh',num2str(nVeh),'_T',num2str(T_end),'_',priority_option];
    
                if isParl
                    results_name = [results_name,'_maxCLs',num2str(max_num_CLs),...
                        '_ConsiderVehWithoutROW',strategy_consider_veh_without_ROW,'_EnterLaneletCrossingArea',strategy_enter_lanelet_crossing_area];
                end
            else
                % use custom name 
                results_name = customResultName;
            end

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