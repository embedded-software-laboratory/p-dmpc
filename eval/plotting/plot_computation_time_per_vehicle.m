function plot_computation_time_per_vehicle(experiment_result, optional)

    arguments
        experiment_result (1, 1) ExperimentResult;
        optional.do_export (1, 1) logical = true;
        optional.fig (1, 1) matlab.ui.Figure = figure(visible = "on");
        optional.export_fig_cgf (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    computation_time = data_time_experiment(experiment_result);
    computation_time = computation_time';

    bar(1:size(computation_time, 1), computation_time);
    xlabel('Time step', Interpreter = 'LaTex');
    ylabel('Computation Time [s]', Interpreter = 'LaTex');

    if optional.do_export
        results_folder = FileNameConstructor.experiment_result_folder_path(experiment_result.options);
        filepath = fullfile(results_folder, 'computation_time_per_vehicle.pdf');
        set_figure_properties(optional.fig, ExportFigConfig.paper('paperheight', 12))
        export_fig(optional.fig, filepath);
        if (~optional.fig.Visible); close(optional.fig); end
    end

end
