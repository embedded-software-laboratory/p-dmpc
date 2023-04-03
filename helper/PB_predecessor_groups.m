function [groups] = PB_predecessor_groups(topological_grouping_matrix)
    % PB_PREDECESSOR_GROUPS   Returns the members and predecessors based on the topolical levels.

    groups = struct;

    for group_idx = 1:size(topological_grouping_matrix, 1)
        groups(group_idx).members = find(topological_grouping_matrix(group_idx, :));

        if group_idx == 1
            groups(group_idx).predecessors = [];
        else
            groups(group_idx).predecessors = [groups(group_idx - 1).predecessors groups(group_idx - 1).members];
        end

    end

end
