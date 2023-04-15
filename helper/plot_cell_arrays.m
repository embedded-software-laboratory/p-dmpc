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
    
    for j = size(cells,2):-1:1
        if isFill
            faceAlpha = (1-j/(size(cells,2)+2))*0.5;
            edgeColor = 'k';
        else
            faceAlpha = 0;
            edgeColor = color;
        end
        shape = cells{j};
        if isa(shape, 'polyshape')
            plot(shape,'FaceColor',color,'EdgeColor',edgeColor,'FaceAlpha',faceAlpha);
        else
            patch(shape(1,:),shape(2,:),color,'EdgeColor',edgeColor,'FaceAlpha',faceAlpha);
        end
    end

end