classdef SimLab < InterfaceExperiment
% SIMLAB    Instance of experiment interface used for simulation in matlab.
    
    properties (Access=private)
        doExploration
        plotter % own plotter to visualize if no visualization_data_queue is given
        visualization_data_queue
        use_visualization_data_queue
        doOnlinePlot
    end
    
    methods
        function obj = SimLab(scenario, visualization_data_queue)
            obj.scenario = scenario;
            obj.doOnlinePlot = obj.scenario.options.visu(1);
            obj.doExploration = obj.scenario.options.visu(2);
            obj.use_visualization_data_queue = false;    

            obj.cur_node = node(0, [obj.scenario.vehicles(:).trim_config], [obj.scenario.vehicles(:).x_start]', [obj.scenario.vehicles(:).y_start]', [obj.scenario.vehicles(:).yaw_start]', zeros(obj.scenario.options.amount,1), zeros(obj.scenario.options.amount,1));
            
            if ~isempty(visualization_data_queue)
                obj.visualization_data_queue = visualization_data_queue;
                obj.use_visualization_data_queue = true;
            end
            if obj.doOnlinePlot
                if obj.use_visualization_data_queue
                    TODO
                else
                    obj.plotter = PlotterOnline(obj.scenario.options.optionsPlotOnline);
                end
            end
        end
        
        function setup(obj)
        end

% TODO  function still in use?     
%         function update(obj)
%             obj.cur_node = node(0, [obj.scenario.vehicles(:).trim_config], [obj.scenario.vehicles(:).x_start]', [obj.scenario.vehicles(:).y_start]', [obj.scenario.vehicles(:).yaw_start]', zeros(obj.scenario.options.amount,1), zeros(obj.scenario.options.amount,1));
%         end
        
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
                pause(obj.scenario.options.dt-result.step_time(obj.k))

                % visualize time step
                % tick_now = obj.scenario.options.tick_per_step + 2; % plot of next time step. set to 1 for plot of current time step
                tick_now = 1; % plot of next time step. set to 1 for plot of current time step
                obj.plotter.plotOnline(result, obj.k, tick_now, exploration_struct);
            else
                % pause so that `keyPressCallback()` can be executed in time
                pause(0.01)
            end
        end
        
        function got_stop = is_stop(obj)
            got_stop = false;
            % idle while paused, and check if we should stop early
            while obj.plotter.paused
                if obj.plotter.abort
                    disp('Aborted.');
                    got_stop = true;
                    break;
                end
                pause(0.1);
            end
            if obj.plotter.abort
                disp('Aborted.');
                got_stop = true;
            end
            if  obj.k >= obj.scenario.options.k_end
                disp('Simulation will be stopped as the defined simulation duration is reached.')
                got_stop = true;
            end
        end
        
        function end_run(obj)
            disp('End')
            if obj.doOnlinePlot && ~obj.use_visualization_data_queue
                obj.plotter.close_figure();
            end
        end
    end
end

