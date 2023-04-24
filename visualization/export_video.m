function export_video(result, videoExportSetup)
    % EXPORTVIDEO   Export Video from results of simulation.
    scenario = result.scenario;
    nSteps = nnz(result.controller_runtime);

    if nargin > 1
        framerate = videoExportSetup.framerate;
        frame_ticks = 1;
    else
        framerate = 30;
        frame_per_step = framerate * scenario.options.dt;
        frame_ticks = round(linspace(2, scenario.options.tick_per_step + 1, frame_per_step));
    end

    plotter = PlotterOnline(scenario);
    plotter.set_figure_visibility(false);

    test_mode = false;

    if test_mode
        plotting_info = PlottingInfo(scenario.options.veh_ids, result, 1, 1);
        plotter.plot(plotting_info);
        set_figure_properties(plotter.get_figure(), ExportFigConfig.video());
        frame = getframe(plotter.get_figure());
        imwrite(frame.cdata, ['output\video_', scenario.options.scenario_name, '.png']);
        return
    end

    v = VideoWriter( ...
        FileNameConstructor.gen_video_file_path(scenario.options), ...
        'Motion JPEG AVI' ...
    );
    v.FrameRate = framerate;
    v.Quality = 97;
    open(v);

    startTimer = tic;

    disp('Exporting video ...');
    wb = waitbar(0, 'Exporting video ...', 'Name', 'Video Export Progress');

    scenario.options.options_plot_online.is_video_mode = 1;

    for step_idx = 1:nSteps

        for frame_idx = frame_ticks
            plotting_info = PlottingInfo(scenario.options.veh_ids, result, step_idx, frame_idx);
            plotter.plot(plotting_info);
            set_figure_properties(plotter.get_figure(), ExportFigConfig().video);
            frame = getframe(plotter.get_figure());
            writeVideo(v, frame);
            progress = (find(frame_ticks == frame_idx) / length(frame_ticks)) ...
                * (1 / nSteps) + ((step_idx - 1) / nSteps);
            ETA = toc(startTimer) * (1 - progress) / progress;
            waitbar(progress, wb, ...
                sprintf('Exporting video, %4.1f sec remaining...', ETA) ...
            );
        end

    end

    plotter.close_figure();
    close(wb);
    close(v);

end
