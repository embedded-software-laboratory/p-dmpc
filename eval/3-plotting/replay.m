function replay(experiment_result)
    %REPLAY Replay the ExperimentResult step by step.
    %   Right key moves one step forward, left key moves one step backward.
    arguments
        experiment_result (1, 1) ExperimentResult;
    end

    plotter = PlotterOffline(experiment_result);
    plotter.start();
end
