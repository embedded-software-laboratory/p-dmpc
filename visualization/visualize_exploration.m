function plotInfo = visualize_exploration(scenario, info, plotInfo)
    % nNodes = info.tree.nnodes;
    % for iVeh = 1:scenario.nVeh
    %     plotInfo.search(iVeh).XData = zeros(1,nNodes);
    %     plotInfo.search(iVeh).YData = zeros(1,nNodes);
    %     plotInfo.search(iVeh).ZData = zeros(1,nNodes);
    % end
    
    % for iNode = info.tree.depthfirstiterator
    %     for iVeh = 1:scenario.nVeh
    %         plotInfo.search(iVeh).XData(iNode) = info.tree.Node{iNode}(iVeh,info.tree.idx.x);
    %         plotInfo.search(iVeh).YData(iNode) = info.tree.Node{iNode}(iVeh,info.tree.idx.y);
    %         plotInfo.search(iVeh).ZData(iNode) = ...
    %             sum(info.tree.Node{iNode}(:,[info.tree.idx.g,info.tree.idx.h]),[1,2]);
    %     end
    % end

    t = [info.tree.Node{:}];
    t = reshape(t,2,8,[]);
    leaf_idcs = find(t(1,info.tree.idx.depth,:)==6);
    leaves = t(:,:,leaf_idcs);
    t(:,:,leaf_idcs) = [];
    % plotInfo.open.XData = reshape(t(:,info.tree.idx.x,:),1,[]);
    % plotInfo.open.YData = reshape(t(:,info.tree.idx.y,:),1,[]);
    % plotInfo.open.ZData = kron(reshape(sum(t(:,[info.tree.idx.g,info.tree.idx.h],:),[1,2]),1,[]),[1,1]);
    plotInfo.open_leaves.XData = reshape(leaves(:,info.tree.idx.x,:),1,[]);
    plotInfo.open_leaves.YData = reshape(leaves(:,info.tree.idx.y,:),1,[]);
    plotInfo.open_leaves.ZData = kron(reshape(sum(leaves(:,[info.tree.idx.g,info.tree.idx.h],:),[1,2]),1,[]),[1,1]);
end