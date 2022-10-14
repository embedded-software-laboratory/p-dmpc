function [ res ] = eval_levels(res)
    [ nVeh, nPri, nSce ] = size(res);
    nLevels_by_veh_pri = cell(nVeh, nPri);
    nLevels_by_pri = cell(nPri);
    for iVeh = 1:nVeh
        for iPri = 1:nPri
            for iSce = 1:nSce
                result = res{iVeh,iPri,iSce};
                % get number of steps until deadlock
                nSteps = floor(result.t_total/(result.scenario.options.dt))-1;
                % get number of levels by max priority assigned
                result.nLevels = max(result.priority(:,1:nSteps));
                res{iVeh,iPri,iSce} = result;
                nLevels_by_veh_pri{iVeh,iPri} = [nLevels_by_veh_pri{iVeh,iPri} result.nLevels];
                nLevels_by_pri{iPri} = [nLevels_by_pri{iPri} result.nLevels];
            end
        end
    end
    % fill rows with zeros
    nLevels_by_pri_mat = [];
    max_entries = max(cellfun(@length,nLevels_by_pri));
    for iPri = 1:nPri
        tmp = nLevels_by_pri{iPri};
        tmp(end+1:max_entries) = 0;
        nLevels_by_pri{iPri} = tmp;
        nLevels_by_pri_mat = [nLevels_by_pri_mat; nLevels_by_pri{iPri}];
    end
    % plot
    figHandle = figure();
    hist(nLevels_by_pri_mat',0:6);
    xlim([0.5, 6.5]);
    h_legend = legend('FCA','Random','Constant','Coloring');
    h_xlabel = xlabel('$n_{CLs}$','Interpreter','latex');
    h_ylabel = ylabel('# of occurences','Interpreter','latex');
    
    % set properties
    fontsize    = 9;
    paperwidth  = 10;    % picture width in cm
    paperheight = 10;    % picture height in cm
    linewidth   = 0.5;
    fontname    = 'CMU Serif';
    units       = 'centimeters';
    set(h_legend,...
                'LineWidth',linewidth,...
                'FontName',fontname,...
                'Interpreter','latex',...
                'Box','on');
    set(h_xlabel,...
                'FontSize',fontsize,...
                'FontName',fontname,...
                'Interpreter','latex')
    set(h_ylabel,...
                'FontSize',fontsize,...
                'FontName',fontname,...
                'Interpreter','latex')
    % background color
    set(figHandle, 'Color', 'w');
    % format
    set(figHandle,'Units',units);
    screenpos = get(figHandle,'Position');
    set(figHandle ...
        ,'Position',[screenpos(1:2), paperwidth, paperheight]...  % px, py, w, h, of figure on screen
    );
end

