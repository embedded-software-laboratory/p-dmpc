function exportVideo( result )

close all;
clc;
framerate = 30;
resolution = [1920 1080];

tick_skip = round(1/(0.4/49)/framerate);

frame_ticks = 1:tick_skip:49;

figure('Visible','Off','Color',[1 1 1],'units','pixel','OuterPosition',[100 100 resolution(1)/2 resolution(2)/2]);

% tune resolution
plotOnline(result,1,1);
frame = export_fig(gcf, '-nocrop', '-a4', '-m2');
set(gcf,'OuterPosition',[100 100 resolution(1)-size(frame,2)/2 resolution(2)-size(frame,1)/2]);

[~,vid_name,~] = fileparts(fileparts('output\UnnamedScenario_test\'));

test_mode = false;
if test_mode
    frame = export_fig(gcf, '-nocrop', '-a4', '-m2');
    imwrite(frame,['output\video_' vid_name '.png']);
    return
end

v = VideoWriter(['output\video_' vid_name '.mp4'],'MPEG-4');
v.FrameRate = framerate;
v.Quality = 97;
open(v);

startTimer = tic;

for step_idx = 1:19
    for frame_idx = frame_ticks
        clf
        plotOnline(result,step_idx,frame_idx);
        frame = export_fig(gcf, '-nocrop', '-a4', '-m2');
        writeVideo(v,frame);
        clc;
        progress = (find(frame_idx)/length(frame_ticks))*(step_idx/19);
        progress_bar(1:50) = ' ';
        progress_bar(1:round(50*progress)) = '=';
        progress_bar(max(1,round(50*progress))) = '>';
        fprintf('|%s|\n', progress_bar);
        fprintf('%2.3f %%\n', 100*progress);
        ETA = toc(startTimer)*(1-progress)/progress;
        fprintf('ETA: %4.1f sec\n', ETA);    
    end
end
close(v);

end

