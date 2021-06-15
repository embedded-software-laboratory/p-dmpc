% MIT License
% 
% Copyright (c) 2021 Lehrstuhl Informatik 11 - RWTH Aachen University
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% 
% This file is part of receding-horizon-graph-search.
% 
% Author: i11 - Embedded Software, RWTH Aachen University

function exportVideo( result )

framerate = 30;
resolution = [1920 1080];
scenario = result.scenario;
nSteps = nnz(result.controller_runtime);
frame_per_step = framerate*scenario.dt;

frame_ticks = round(linspace(2,scenario.tick_per_step+1,frame_per_step));

fig = figure('Visible','Off'...
            ,'Color',[1 1 1]...
            ,'units','pixel'...
            ,'OuterPosition',[100 100 resolution(1)/2 resolution(2)/2]...
);



[~,vid_name,~] = fileparts(result.output_path);

test_mode = false;
if test_mode
    plotOnline(result,1,1,[]); %#ok<UNRCH>
    set_figure_properties(fig,'video');
    frame = getframe(fig);
    imwrite(frame,['output\video_', vid_name, '.png']);
    return
end
vidname = ['video_' vid_name '.mp4'];
v = VideoWriter(fullfile(result.output_path,vidname),'MPEG-4');
v.FrameRate = framerate; 
v.Quality = 97;
open(v);

startTimer = tic;

progress_bar(1:50) = ' ';
for step_idx = 1:nSteps
    for frame_idx = frame_ticks
        clf
        plotOnline(result,step_idx,frame_idx,[]);
        set_figure_properties(fig,'video');
        frame = getframe(fig);
        writeVideo(v,frame);
        clc;
        progress = (find(frame_ticks==frame_idx)/length(frame_ticks))*(1/nSteps)+((step_idx-1)/nSteps);
        progress_bar(1:ceil(50*progress)) = '=';
        progress_bar(ceil(50*progress)) = '>';
        fprintf('|%s|\n', progress_bar);
        fprintf('%2.3f %%\n', 100*progress);
        ETA = toc(startTimer)*(1-progress)/progress;
        fprintf('ETA: %4.1f sec\n', ETA);
    end
end
close(v);
close(fig);

end