function p = plot_cell_arrays(cells,color,isFill)
% PLOT_CELL_ARRAYS Plot shapes contained in a cell array
% Inputs
%   cells: cell array
%   color: color of the plot
%   isFill: true/false, if true, the plotted polygons will be filled with
%   color

    if nargin==1
        isFill = false;
    end
    if isFill
        for j = size(cells,2):-1:1
            shape = cells{j};
            patch(shape(1,:),shape(2,:),color,'FaceAlpha',(1-j/(size(cells,2)+2))*0.5);
        end
    else
        CM = rwth_color_order(size(cells,2)+3);
        CM = CM(4:end,:);
    
        for j = 1:size(cells,2)
            shape = cells{j};
            p(j) = plot(shape(1,:),shape(2,:),'LineWidth',1,'Color',CM(j,:),'LineStyle','-.');
        end
    end
end