function plotInfo = visualize_exploration(scenario, theTree, plotInfo)
    
    nNodes = theTree.nnodes;
    for iVeh = 1:scenario.nVeh
        plotInfo.openNodes(iVeh).XData = zeros(1,nNodes);
        plotInfo.openNodes(iVeh).YData = zeros(1,nNodes);
    end
    % labels = cell(scenario.nVeh,nNodes);

    for iNode = theTree.breadthfirstiterator
        for iVeh = 1:scenario.nVeh
            % if ~any(ismember(horizon{1}(j).XData, node.xs(j))) || ~any(ismember(horizon{1}(j).YData, node.ys(j)))
            % labels{iVeh,iNode} = text(theTree.Node{iNode}.xs(iVeh)+0.1, theTree.Node{iNode}.ys(iVeh)-0.1, num2str(iNode), 'FontSize', 6);
            plotInfo.openNodes(iVeh).XData(iNode) = theTree.Node{iNode}.xs(iVeh);
            plotInfo.openNodes(iVeh).YData(iNode) = theTree.Node{iNode}.ys(iVeh);
            % end
        end
    end
    % for iVeh = 1:scenario.nVeh
    %     c = vehColor(iVeh);
    %     plotNodes{iVeh} = scatter(xdata(iVeh,:), ydata(iVeh,:) ...
    %         ,'LineWidth', 3 ...
    %         ,'MarkerEdgeColor', c ...
    %         ,'MarkerFaceColor', c ...
    %         ,'MarkerEdgeAlpha', 0.4 ...
    %         ,'MarkerFaceAlpha', 0.4 ...
    %     );
    % end
    drawnow
end