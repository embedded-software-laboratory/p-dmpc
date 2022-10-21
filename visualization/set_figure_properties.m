function set_figure_properties(figHandle, options)
% SET_FIGURE_PROPERTIES     Set Properties used for figures based on the type of export.
arguments
    figHandle       (1,1) matlab.ui.Figure;
    options.preset;
    options.paperheight_in  (1,1) double;
    options.paperwidth_in   (1,1) double;
end

fontsize    = 9;
markersize  = 3;
linewidth   = 0.5;
fontname    = 'CMU Serif';
units       = 'centimeters';
paperwidth  = 7.5;    % picture width in cm
paperheight = 4;      % picture height in cm

if isfield(options, 'preset')
    switch lower(options.preset)
    case 'paper' % \the\linewidth=252.0pt, 1pt=0.3515mm --> 88.578mm
        fontsize    = 9;
        paperwidth  = 7.5;    % picture width in cm
        paperheight = 4;    % picture height in cm
        linewidth   = 0.5;
        fontname    = 'CMU Serif';
        units       = 'centimeters';
    
    case 'presentation'
        fontsize    = 18;
        paperwidth  = 31.77; % picture width in cm
        paperheight = 14.01; % picture height in cm
        linewidth   = 1;
        fontname    = 'Arial';
        units       = 'centimeters';
    
    case 'document'
        fontsize    = 9;
        paperwidth  = 11.7; % picture width in cm
        paperheight = 7.85; % picture height in cm
        linewidth   = 0.5;
        fontname    = 'CMU Serif';
        units       = 'centimeters';

    case 'video'
        fontsize    = 20;
        paperwidth  = 1920;
        paperheight = 1080;
        linewidth   = 1;
        fontname    = 'Arial';
        units       = 'pixels';
    
    otherwise % default
        error('No valid preset selected.')
    end
end

if isfield(options, 'paperheight_in')
    paperheight = options.paperheight_in;
end

if isfield(options, 'paperwidth_in')
    paperwidth = options.paperwidth_in;
end
    
    % beauty corrections
    allchildren = get(figHandle, 'Children'); % get handle figure
    for a=1:length(allchildren)
        try % set title handle
            h_title=get(allchildren(a),'Title');
            set(h_title,...
                'FontWeight','normal',...
                'FontSize',fontsize+1,...
                'FontName',fontname,...
                'Interpreter','latex');
        catch
            % continue
        end
        try % redefine x- and y-labels
            h_xlabel = get(allchildren(a), 'xlabel');
            h_ylabel = get(allchildren(a), 'ylabel');
            set(h_xlabel,...
                'FontSize',fontsize,...
                'FontName',fontname,...
                'Interpreter','latex')
            set(h_ylabel,...
                'FontSize',fontsize,...
                'FontName',fontname,...
                'Interpreter','latex')
        catch
            % continue
        end
        % set axes
        try
            h_axes=get(allchildren(a),'Axes');
            set(h_axes,...
                'FontSize',fontsize,...
                'FontName',fontname,...
                'LineWidth',linewidth, ...
                'Box','on');
        catch
            % continue
        end
        % set subplotaxes
        try
            set(allchildren(a)...
                ,'FontSize',fontsize...
                ,'FontName',fontname...
                ,'LineWidth',linewidth...
                ,'Box','on'...
            );
                % ,'XAxisLocation','origin'...
                % ,'YAxisLocation','origin'...
        catch
            % continue
        end
        % set legend
        if strcmpi(get(allchildren(a),'Tag'),'legend')
            h_legend=allchildren(a);
            set(h_legend,'FontSize',fontsize-3)
            set(h_legend,...
                'LineWidth',linewidth,...
                'FontName',fontname,...
                'Interpreter','latex',...
                'Box','on');
        end
        % Set graphic objects
        
        h_graphics = get(allchildren(a),'Children');
        for h_graphic = h_graphics'
            try
                set(h_graphic ...
                    ,'LineWidth', linewidth ...
                );
            catch
                % continue
            end
            try
                set(h_graphic ...
                    ,'MarkerSize', markersize ...
                );
            catch
                % continue
            end
        end
    end
    
    % background color
    set(figHandle, 'Color', 'w');
    % format
    set(figHandle,'Units',units);
    screenpos = get(figHandle,'Position');
    set(figHandle ...
        ,'Position',[screenpos(1:2), paperwidth, paperheight]...  % px, py, w, h, of figure on screen
    );

    colororder(rwth_color_order());
end