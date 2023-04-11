function level = computation_level_members(topological_grouping_matrix)
    % PB_PREDECESSOR_GROUPS   Returns the members and predecessors based on the topolical levels.

    level = struct;

    for group_idx = 1:size(topological_grouping_matrix, 1)
        level(group_idx).members = find(topological_grouping_matrix(group_idx, :));

        if group_idx == 1
            level(group_idx).predecessors = [];
        else
            level(group_idx).predecessors = [level(group_idx - 1).predecessors level(group_idx - 1).members];
        end

    end

end
