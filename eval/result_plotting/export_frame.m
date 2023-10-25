function export_frame(result, frame_export_setup)
    % EXPORTFRAME   Export frame(s) from results of simulation.
    % INPUT
    %   result: simulation result
    %
    arguments
        result struct;
        frame_export_setup.iStep {mustBeInteger} = 1;
        frame_export_setup.frame_name string = 'frame.png';
    end

    close all % if not, sometimes color bar will not be shown properly
    scenario = result.scenario;

    plotter = PlotterOnline(scenario);
    plotter.set_figure_visibility(false);
    frame_idx = 1;
    plotting_info = PlottingInfo(result.vehicle_ids, result, frame_export_setup.iStep, frame_idx);
    plotter.plot(plotting_info);
    set_figure_properties(plotter.get_figure(), ExportFigConfig.video());
    frame = getframe(plotter.get_figure());

    results_folder_path = FileNameConstructor.all_results();

    filepath = fullfile(results_folder_path, frame_export_setup.frame_name); % full path of the fig

    imwrite(frame.cdata, filepath);
    disp(append('A figure was saved under ', filepath))

    plotter.close_figure();

end
