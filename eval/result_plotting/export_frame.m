function export_frame(result, optional)
    % EXPORTFRAME   Export frame(s) from results of simulation.
    % INPUT
    %   result: simulation result
    %
    arguments
        result struct;
        optional.iStep {mustBeInteger} = 1;
        optional.frame_name string = 'frame.png';
    end

    close all % if not, sometimes color bar will not be shown properly
    options = result.options;
    scenario = result.scenario;

    plotter = PlotterOnline(options, scenario);
    plotter.set_figure_visibility(false);
    frame_idx = 1;
    plotting_info = PlottingInfo(result.vehicle_ids, result, optional.iStep, frame_idx);
    plotter.plot(plotting_info);
    set_figure_properties(plotter.get_figure(), ExportFigConfig.video());
    frame = getframe(plotter.get_figure());

    results_folder_path = FileNameConstructor.all_results();

    filepath = fullfile(results_folder_path, optional.frame_name); % full path of the fig

    imwrite(frame.cdata, filepath);

    plotter.close_figure();

end
