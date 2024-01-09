function plot_computation_time(experiment_results, optional)

    arguments
        experiment_results (:, :) ExperimentResult;
        optional.do_export (1, 1) logical = true;
        optional.fig (1, 1) matlab.ui.Figure = figure(visible = "off");
        optional.export_fig_cfg (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    experiment_result = merge_experiment_results(experiment_results);

    plot_computation_time_per_vehicle(experiment_result, do_export = optional.do_export, export_fig_cgf = optional.export_fig_cfg);

    plot_computation_time_of_all_vehicles(experiment_results, do_export = optional.do_export, export_fig_cgf = optional.export_fig_cfg);

end
