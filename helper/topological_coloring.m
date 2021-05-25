function [valid,L] = topological_coloring(A)
    % init
    a = length(A(:,1));
    col = 1:a;
    color = zeros(1,a);
    cum_matrix = cumsum(A);
    degree = cum_matrix(end,:);
    % assignment
    color(degree == 0) = 1;
    while ~all(color ~= 0)
        % get next node in coloring order
        node = next_node_sdo_ldo(A,color,degree);
        neighbor_col  = unique(color(A(node,:)));
        poss_col = setdiff(col,neighbor_col);
        color(node) = poss_col(1);
    end
    % topolical sorting matrix
    used_col = unique(color);
    k_col = length(used_col);
    L = zeros(k_col,a);
    for i = 1 : k_col
        L(i,color == used_col(i)) = 1;
    end
    valid = isequal(sum(sum(L,1)),a);
end

