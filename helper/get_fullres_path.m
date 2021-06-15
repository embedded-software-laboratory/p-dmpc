function [ fullres_path ] = get_fullres_path(info,scenario)

    if isstruct(info)
        fullres_path = path_between(info.tree.node{info.tree_path(1)},info.tree.node{info.tree_path(2)},info.tree,scenario.mpa);
        return
    end
    fullres_path = cell(scenario.nVeh,1);
    for i = 1:scenario.nVeh
        fullres_path(i) = path_between(info{i}.tree.node{info{i}.tree_path(1)},info{i}.tree.node{info{i}.tree_path(2)},info{i}.tree,scenario.mpa);
    end
    
end

