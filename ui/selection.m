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

function options = selection(vehicle_amount,plot_option,strat_option)
%% Specify amount of vehicles
scenarios = {
    '1', pi+pi; ...
    '2', pi+pi*(1:2); ...
    '3', pi+2*pi/3*(1:3); ...
    '4', pi+2*pi/4*(1:4); ...
    '5', pi+2*pi/5*(1:5); ...
    '6', pi+2*pi/6*(1:6); ...
    '7', pi+2*pi/7*(1:7); ...
    '8', pi+2*pi/8*(1:8); ...
    };

possPlots = {
    '1', 'no visualization',                    [false,false]; ...
    '2', 'vehicle visualization',               [true,false]; ...
    '3', 'visualization + node exploration',    [true,true]; ... % only for centralized controller
    };

possStrats = {
    '1', 'centralized'; ...
    '2', 'pb non-coop'; ...
    };



    % ====== load previous choice ======
    try
        load([tempdir 'uiSelection'], 'vehicle_amount', 'plot_option', 'strat_option');
    catch
        % continue
    end

if nargin < 1
    if ~exist('vehicle_amount','var')
        vehicle_amount = 1;
    end
    if ~exist('plot_option','var')
        plot_option = 1;
    end
    if ~exist('strat_option','var')
        strat_option = 1;
    end
    
    [strat_option,ok] = listdlg(...
        'ListString',possStrats(:,2), ...
        'ListSize', [300,300], ...
        'SelectionMode', 'single', ...
        'InitialValue', strat_option, ...
        'PromptString', 'Choose the control strategy');

    if(~ok)
        error('Canceled');
    end
    
    [vehicle_amount,ok] = listdlg(...
        'ListString',scenarios(:,1), ...
        'ListSize', [300,300], ...
        'SelectionMode', 'single', ...
        'InitialValue', vehicle_amount, ...
        'PromptString', 'Choose the amount of vehicles');

    if(~ok)
        error('Canceled');
    end
    
    [plot_option,ok] = listdlg(...
        'ListString',possPlots(1:end-(strat_option-1),2), ... % remove option if controller is pbnc
        'ListSize', [300,300], ...
        'SelectionMode', 'single', ...
        'InitialValue', plot_option, ...
        'PromptString', 'Choose the type of visualization during simulation');

    if(~ok)
        error('Canceled');
    end
    
end

options.isPB = (strat_option == 2);
options.angles = scenarios{vehicle_amount,2};
options.amount = vehicle_amount;
options.visu = possPlots{plot_option,3};


    % ============= save choice ============
    save([tempdir 'uiSelection'], 'vehicle_amount', 'plot_option', 'strat_option');

end

