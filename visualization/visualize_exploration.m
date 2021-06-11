function visualize_exploration(exploration,scenario)
    info = exploration.info;
    t = [info.tree.node{:}];
    t = reshape(t,scenario.nVeh,8,[]);
    leaf_idcs = find(t(1,NodeInfo.depth,:)==scenario.Hp);
    leaves = t(:,:,leaf_idcs);
    t(:,:,leaf_idcs) = [];
    x = reshape(leaves(:,NodeInfo.x,:),1,[]);
    y = reshape(leaves(:,NodeInfo.y,:),1,[]);
    z = kron(reshape(sum(leaves(:,[NodeInfo.g,NodeInfo.h],:),[1,2]),1,[]),[1,1]);
    line(x,y,z,'Marker','d', 'MarkerSize',6,'LineStyle','none');
end