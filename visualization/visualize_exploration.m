function plotInfo = visualize_exploration(scenario, theTree, plotInfo)
    
    nNodes = theTree.nnodes;
    for iVeh = 1:scenario.nVeh
        plotInfo.search(iVeh).XData = zeros(1,nNodes);
        plotInfo.search(iVeh).YData = zeros(1,nNodes);
        plotInfo.search(iVeh).ZData = zeros(1,nNodes);
    end
    
    for iNode = theTree.depthfirstiterator
        for iVeh = 1:scenario.nVeh
            plotInfo.search(iVeh).XData(iNode) = theTree.Node{iNode}.xs(iVeh);
            plotInfo.search(iVeh).YData(iNode) = theTree.Node{iNode}.ys(iVeh);
            plotInfo.search(iVeh).ZData(iNode) = ...
                theTree.Node{iNode}.f_value;
        end
    end
end