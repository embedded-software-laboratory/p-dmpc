function export_video(result, optional)
    % EXPORTVIDEO   Export Video from results of simulation.
    arguments
        result struct;
        optional.framerate {mustBeNumeric} = 30;
    end

    options = result.options;
    scenario = result.scenario;
    options.options_plot_online.is_video_mode = 1;

    nSteps = result.nSteps;

    if nargin > 1
        framerate = optional.framerate;
        frame_ticks = 1;
    else
        framerate = 30;
        frame_per_step = framerate * options.dt_seconds;
        frame_ticks = round(linspace(2, options.tick_per_step + 1, frame_per_step));
    end

    plotter = PlotterOnline(options, scenario);
    plotter.set_figure_visibility(false);

    v = VideoWriter( ...
        FileNameConstructor.gen_video_file_path(options), ...
        'Motion JPEG AVI' ...
    );
    v.FrameRate = framerate;
    v.Quality = 97;
    open(v);

    startTimer = tic;

    disp('Exporting video ...');
    wb = waitbar(0, 'Exporting video ...', 'Name', 'Video Export Progress');

    for step_idx = 1:nSteps

        for frame_idx = frame_ticks
            plotting_info = PlottingInfo(1:options.amount, result, step_idx, frame_idx);
            plotter.plot(plotting_info);
            frame = getframe(plotter.get_figure());
            writeVideo(v, frame);
        end

        progress = step_idx / nSteps;
        time_remaining_seconds = toc(startTimer) * (1 - progress) / progress;
        waitbar(progress, wb, ...
            sprintf('Exporting video. %4.1f sec remaining...', time_remaining_seconds) ...
        );

    end

    plotter.close_figure();
    close(wb);
    close(v);

end
