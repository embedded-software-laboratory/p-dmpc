classdef SimLab < InterfaceExperiment
% SIMLAB    Instance of experiment interface used for simulation in matlab.
    
    properties (Access=private)
        doOnlinePlot
        doExploration
        fig
        resolution
        paused
        abort
    end

    properties
        visu % struct to store logical variables indicating whether to show vehicle ID/priority/coupling/coupling weights
    end
    
    methods
        function obj = SimLab(scenario, options)
            obj.doOnlinePlot = options.visu(1);
            obj.doExploration = options.visu(2);
            obj.scenario = scenario;

            % variables for key press callback
            obj.paused = false;
            obj.abort = false;

            obj.visu.isShowVehID = true;
            obj.visu.isShowPriority = false;
            obj.visu.isShowCoupling = true;
            obj.visu.isShowWeight = false;
            obj.visu.colormap = []; % n-by-3 matrix containing RGB values
            obj.visu.colorbar = []; % object of the MATLAB class ColorBar

            obj.cur_node = node(0, [obj.scenario.vehicles(:).trim_config], [obj.scenario.vehicles(:).x_start]', [obj.scenario.vehicles(:).y_start]', [obj.scenario.vehicles(:).yaw_start]', zeros(obj.scenario.nVeh,1), zeros(obj.scenario.nVeh,1));
        end
        
        function keyPressCallback(obj, ~, eventdata)
            switch eventdata.Key
                case 'escape'
                    obj.abort = true;
                case 'space'
                    obj.paused = ~obj.paused;
                    if obj.paused 
                        disp('Pause simulation.')
                    else
                        disp('Start simulation.')
                    end
                case 'i'
                    obj.visu.isShowVehID = ~obj.visu.isShowVehID;
                    if obj.visu.isShowVehID 
                        disp('Show vehicle.')
                    else
                        disp('Hide Vehicle IDs.')
                    end
                case 'p'
                    obj.visu.isShowPriority = ~obj.visu.isShowPriority;
                    if obj.visu.isShowPriority 
                        disp('Show vehicle priorities.')
                    else
                        disp('Hide vehicle priorities.')
                    end
                case 'c'
                    obj.visu.isShowCoupling = ~obj.visu.isShowCoupling;
                    if obj.visu.isShowCoupling 
                        disp('Show couplings lines.')
                    else
                        disp('Hide couplings lines.')
                    end
                case 'w'
                    obj.visu.isShowWeight = ~obj.visu.isShowWeight;
                    if obj.visu.isShowWeight 
                        disp('Show couplings weights.')
                    else
                        disp('Hide couplings weights.')
                    end
            end
        end
        
        function setup(obj)
            if obj.doOnlinePlot
                obj.resolution = [1920 1080];
                obj.fig = figure(...
                    'Visible','On'...
                    ,'Color',[1 1 1]...
                    ,'units','pixel'...
                    ,'OuterPosition',[100 100 obj.resolution(1) obj.resolution(2)]...
                );
                set(gcf,'WindowKeyPressFcn',@obj.keyPressCallback);
                hold on
            end
        end

        function update(obj)
            obj.cur_node = node(0, [obj.scenario.vehicles(:).trim_config], [obj.scenario.vehicles(:).x_start]', [obj.scenario.vehicles(:).y_start]', [obj.scenario.vehicles(:).yaw_start]', zeros(obj.scenario.nVeh,1), zeros(obj.scenario.nVeh,1));
        end
        
        function [ x0, trim_indices ] = measure(obj, ~)
            [ x0, trim_indices ] = obj.measure_node();
        end
        
        function apply(obj, info, result, k, ~)
            % simulate change of state
            obj.cur_node = info.next_node;
            obj.k = k;            
            % init struct for exploration plot
            if obj.doExploration
                exploration_struct.doExploration = true;
                exploration_struct.info = info;
            else
                exploration_struct = [];
            end
            if obj.doOnlinePlot
                % wait to simulate realtime plotting
                pause(obj.scenario.dt-result.step_time(obj.k))

                % visualize time step
                tick_now = obj.scenario.tick_per_step + 2; % plot of next time step. set to 1 for plot of current time step
                plotOnline(result, obj.k, tick_now, exploration_struct, obj);
            end
        end
        
        function got_stop = is_stop(obj)
            got_stop = false;
            % idle while paused, and check if we should stop early
            while obj.paused
                if obj.abort
                    disp('Aborted.');
                    got_stop = true;
                    break;
                end
                pause(0.1);
            end
            if obj.abort
                disp('Aborted.');
                got_stop = true;
            end
            if  obj.k >= obj.scenario.k_end
                got_stop = true;
            end
        end
        
        function end_run(obj)
            disp('End')
            if obj.doOnlinePlot
                close(obj.fig)
            end
        end
    end
end

