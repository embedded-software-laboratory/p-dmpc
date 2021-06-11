function visualize_exploration(exploration,scenario)
    info = exploration.info;
    t = [info.tree.node{:}];
    t = reshape(t,scenario.nVeh,8,[]);
    leaf_idcs = find(t(1,info.tree.idx.depth,:)==scenario.Hp);
    leaves = t(:,:,leaf_idcs);
    t(:,:,leaf_idcs) = [];
    x = reshape(leaves(:,info.tree.idx.x,:),1,[]);
    y = reshape(leaves(:,info.tree.idx.y,:),1,[]);
    z = kron(reshape(sum(leaves(:,[info.tree.idx.g,info.tree.idx.h],:),[1,2]),1,[]),[1,1]);
    line(x,y,z,'Marker','d', 'MarkerSize',6,'LineStyle','none');
end