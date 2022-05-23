function plot_reachable_sets_offline(mpa)
%PLOT_REACHABLE_SETS_OFFLINE Visualization of the reachable sets of different root trims
%   s

    n_trims = size(mpa.reachable_sets,1);
    Hp = size(mpa.reachable_sets,2);

    fig = figure('Name','ReachableSets');
    fig.Position = [10 10 500 1600];
    t_fig = tiledlayout(n_trims,Hp,'TileSpacing','compact');
    for i=1:n_trims
        for t=1:Hp
            nexttile;
            plot(mpa.reachable_sets{i,t})
            hold on
            plot(mpa.reachable_sets_conv{i,t})
            if t==1
                ylabel(['root trim ', num2str(i)])
            end
            axis square
            grid on
            xlim([-0.2, 1.5]);
            ylim([-1.4 1.4]);
        end
    end
    xlabel(t_fig,'x [m]')
    ylabel(t_fig,'y [m]')
    title(t_fig,'Reachable Sets of Different Root Trims')
    
    % save fig to pdf
    set(fig,'Units','Inches');
    pos = get(fig,'Position');
    set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    
    [file_path,~,~] = fileparts(mfilename('fullpath')); % get the path of the current file
    idcs = strfind(file_path,filesep); % find all positions of '/'
    one_folder_up = file_path(1:idcs(end)-1); % one folder up
    folder_target = [one_folder_up,filesep,'results',filesep,'reachable_sets_offline'];
    file_name = ['trims',num2str(n_trims),'_Hp',num2str(Hp)];
    full_path = [folder_target,filesep,file_name];
    if isfile(full_path)
        warning('The file for visualization of the offline reachable sets was already saved.');
    else
        print(fig,full_path,'-dpdf','-r0');
    end

end