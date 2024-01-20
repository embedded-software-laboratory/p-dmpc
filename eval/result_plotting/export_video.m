function export_video(experiment_result, optional)
    % EXPORTVIDEO   Export Video from ExperimentResult of simulation.
    arguments
        experiment_result ExperimentResult;
        optional.framerate {mustBeNumeric} = 30;
        optional.step_start {mustBeNumeric} = 1;
        optional.step_end {mustBeNumeric} = experiment_result.n_steps;
    end

    options = experiment_result.options;
    scenario = experiment_result.scenario;
    options.options_plot_online.is_video_mode = 1;

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
    mpa = MotionPrimitiveAutomaton( ...
        BicycleModel( ...
        scenario.vehicles(1).Lf, ...
        scenario.vehicles(1).Lr ...
    ), ...
        options ...
    );

    disp('Exporting video ...');
    wb = waitbar(0, 'Exporting video ...', 'Name', 'Video Export Progress');

    for step_idx = optional.step_start:optional.step_end
        x0 = x0_in_step(step_idx, experiment_result, mpa);

        for frame_idx = frame_ticks
            time_seconds = (step_idx - 1) * options.dt_seconds ...
                + frame_idx * options.dt_seconds / options.tick_per_step;
            plotting_info = PlottingInfo( ...
                1:options.amount, ...
                experiment_result, ...
                step_idx, ...
                time_seconds, ...
                x0(:, :, frame_idx) ...
            );
            plotter.plot(plotting_info);
            frame = getframe(plotter.get_figure());
            writeVideo(v, frame);
        end

        progress = step_idx / optional.step_end;
        time_remaining_seconds = toc(startTimer) * (1 - progress) / progress;
        waitbar(progress, wb, ...
            sprintf('Exporting video. %4.0f sec remaining...', time_remaining_seconds) ...
        );

    end

    plotter.close_figure();
    close(wb);
    close(v);

end

function x0 = x0_in_step(step_idx, experiment_result, mpa)
    i_vehicle_start = 1;
    x0 = nan( ...
        3, ...
        experiment_result.options.amount, ...
        experiment_result.options.tick_per_step + 1 ...
    );

    for control_results_info = experiment_result.control_results_info(:, step_idx)'
        i_vehicle_end = i_vehicle_start + control_results_info.n_vehicles - 1;

        for i_vehicle = i_vehicle_start:i_vehicle_end
            trim_start = experiment_result.iteration_data(step_idx).trim_indices(i_vehicle);
            trim_end = control_results_info.predicted_trims( ...
                i_vehicle - i_vehicle_start + 1, ...
                1 ...
            );
            x = experiment_result.iteration_data(step_idx).x0(i_vehicle, 1);
            y = experiment_result.iteration_data(step_idx).x0(i_vehicle, 2);
            yaw = experiment_result.iteration_data(step_idx).x0(i_vehicle, 3);
            [x0(1, i_vehicle, :), x0(2, i_vehicle, :)] = translate_global( ...
                yaw, ...
                x, ...
                y, ...
                mpa.maneuvers{trim_start, trim_end}.xs, ...
                mpa.maneuvers{trim_start, trim_end}.ys ...
            );
            x0(3, i_vehicle, :) = mpa.maneuvers{trim_start, trim_end}.yaws + yaw;
        end

        i_vehicle_start = i_vehicle_end + 1;
    end

end
