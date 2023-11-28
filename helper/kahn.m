function L = kahn(A)
    % KAHN  Kahn's topological sorting algorithm.

    % init
    a = length(A);
    l = 1; % nr. of levels
    L = zeros(l, a);
    in_D = sum(A, 1);
    old_zeros_in_D = [];

    while ~all(in_D == 1)
        zero_in_D = find(in_D == 0);

        if isempty(zero_in_D)
            valid = false;
            return
        end

        L(l, zero_in_D) = 1;
        l = l + 1;
        A(zero_in_D, :) = 0;
        in_D = sum(A, 1);
        old_zeros_in_D = [old_zeros_in_D zero_in_D];
        in_D(old_zeros_in_D) = 1;
    end

    % check results
    assert(isequal(sum(sum(L, 1)), a));
end
