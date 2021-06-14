function options = selection(vehicle_amount,plot_option)
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
    '3', 'visualization + node exploration',    [true,true]; ...
    };



    % ====== load previous choice ======
    try
        load([tempdir 'uiSelection'], 'vehicle_amount', 'plot_option');
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

    [vehicle_amount,ok] = listdlg(...
        'ListString',scenarios(:,1), ...
        'SelectionMode', 'single', ...
        'InitialValue', vehicle_amount, ...
        'PromptString', 'Choose the amount of vehicles');

    if(~ok)
        error('Canceled');
    end
    
    [plot_option,ok] = listdlg(...
        'ListString',possPlots(:,2), ...
        'SelectionMode', 'single', ...
        'InitialValue', plot_option, ...
        'PromptString', 'Choose the type of visualization during simulation');

    if(~ok)
        error('Canceled');
    end
    
end

options.angles = scenarios{vehicle_amount,2};
options.amount = vehicle_amount;
options.visu = possPlots{plot_option,3};


    % ============= save choice ============
    save([tempdir 'uiSelection'], 'vehicle_amount', 'plot_option');

end

