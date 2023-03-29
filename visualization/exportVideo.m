function exportVideo(result,videoExportSetup)
% EXPORTVIDEO   Export Video from results of simulation.
resolution = [1920 1080];
scenario = result.scenario;
nSteps = nnz(result.controller_runtime);

if nargin>1
    framerate = videoExportSetup.framerate;
    frame_ticks = 1;
else
    framerate = 30;
    frame_per_step = framerate*scenario.options.dt;
    frame_ticks = round(linspace(2,scenario.options.tick_per_step+1,frame_per_step));
end

test_mode = false;
if test_mode
    exp.k = 1; %#ok<UNRCH>
    plotOnline(result,1,1,[],scenario.options.options_plot_online);
    set_figure_properties(fig,ExportFigConfig.video());
    frame = getframe(fig);
    imwrite(frame,['output\video_', vid_name, '.png']);
    return
end
v = VideoWriter(...
    FileNameConstructor.gen_video_file_path(result.scenario.options), ...
    'Motion JPEG AVI' ...
);
v.FrameRate = framerate; 
v.Quality = 97;
open(v);

startTimer = tic;

disp('Exporting video ...');
wb = waitbar(0, 'Exporting video ...','Name','Video Export Progress');

scenario.options.options_plot_online.is_video_mode = 1;

plotter = PlotterOnline(scenario);
plotter.set_figure_visibility(false);

for step_idx = 1:nSteps
    for frame_idx = frame_ticks
        plotting_info = PlottingInfo(scenario.options.veh_ids,result,step_idx,frame_idx,scenario.options.options_plot_online);
        plotter.plotOnline(plotting_info);
        set_figure_properties(plotter.get_figure(),ExportFigConfig().video);
        frame = getframe(plotter.get_figure());
        writeVideo(v,frame);
        progress = ( find(frame_ticks==frame_idx) / length(frame_ticks) )...
            * (1/nSteps) + ( (step_idx-1)/nSteps );
        ETA = toc(startTimer)*(1-progress)/progress;
        waitbar(progress,wb, ...
                sprintf('Exporting video, %4.1f sec remaining...', ETA) ...
        );
    end
end
plotter.close_figure();
close(wb);
close(v);

end
