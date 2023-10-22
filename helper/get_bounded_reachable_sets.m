function [reachable_sets_bounded] = get_bounded_reachable_sets(reachable_sets, lanelet_boundary)
    % bound_reachable_sets calculate the intersection between the reachable sets
    % and the boundary of the lanelets
    %
    % Output:
    %   reachable_sets: cell (1, Hp), reachable sets in the prediction horizon

    arguments
        % reachable set for each step in the prediction horizon
        reachable_sets (1, :) cell % (1, Hp)
        % boundary of the lanelets as polyshape
        lanelet_boundary (1, 1) polyshape
    end

    reachable_sets_bounded = cell(size(reachable_sets));

    Hp = length(reachable_sets);

    for t = 1:Hp

        % compute intersection between reachable set and lanelet boundaries
        reachable_sets_bounded{t} = intersect(reachable_sets{t}, lanelet_boundary);

        if reachable_sets_bounded{t}.NumRegions > 1
            % remove unexpected small regions resulting from computing
            % the intersection

            % sort polyshape intersection by number of sides and
            % separate each region of the polyshape into single polyshape
            % objects (a polyshape object could contain multiple regions)
            polyshape_regions = regions(sortregions( ...
                reachable_sets_bounded{t}, ...
                'numsides', ...
                'descend' ...
            ));
            % take polyshape with the highest number of sides
            reachable_sets_bounded{t} = polyshape_regions(1);
        end

        if isempty(reachable_sets_bounded{t}.Vertices)
            % empty reachable set due to intersection with wrong
            % lanelet boundary

            % restore reachable set
            reachable_sets_bounded{t} = reachable_sets{t};
        end

    end
