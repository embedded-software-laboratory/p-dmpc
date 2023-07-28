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

    % get the path of the target folder to store figures
    [self_path, ~, ~] = fileparts(mfilename('fullpath')); % get the path of the current file
    idcs = strfind(self_path, filesep); % find all positions of '/'
    main_folder = self_path(1:idcs(end) - 1); % one folders up, i.e., to main folder
    results_folder_path = fullfile(main_folder, 'results'); % results folder

    if ~isfolder(results_folder_path)
        % create target folder if not exist
        mkdir(results_folder_path)
    end

    filepath = fullfile(results_folder_path, frame_export_setup.frame_name); % full path of the fig

    imwrite(frame.cdata, filepath);
    disp(append('A figure was saved under ', filepath))

    plotter.close_figure();

end
