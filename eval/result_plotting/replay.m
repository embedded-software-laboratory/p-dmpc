function replay(result)
    %REPLAY Replay the results step by step.
    %   Right key moves one step forward, left key moves one step backward.

    plotter = PlotterOffline(result);
    plotter.start();
end
