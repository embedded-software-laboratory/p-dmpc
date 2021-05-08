function plot_exploration_nodes(exploration,scenario)
        info = exploration.info;
        t = [info.tree.Node{:}];
        t = reshape(t,scenario.nVeh,8,[]);
        leaf_idcs = find(t(1,info.tree.idx.depth,:)==scenario.Hp);
        leaves = t(:,:,leaf_idcs);
        t(:,:,leaf_idcs) = [];
        exp = line(NaN,NaN,NaN, 'Marker','d', 'MarkerSize',6,'LineStyle','none');
        exp.XData = reshape(leaves(:,info.tree.idx.x,:),1,[]);
        exp.YData = reshape(leaves(:,info.tree.idx.y,:),1,[]);
        exp.ZData = kron(reshape(sum(leaves(:,[info.tree.idx.g,info.tree.idx.h],:),[1,2]),1,[]),[1,1]);
end

