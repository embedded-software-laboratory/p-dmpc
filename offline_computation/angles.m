function angles = angles
    load('MotionGraph.mat', 'motion_graph');

    angles = 0;
    trims = 1;
    leafs = 1;
    while ~isempty(leafs)
        for leaf = leafs
            for i = 2:length(motion_graph.motionGraphList(1).trims)
                displacement = round(motion_graph.motionGraphList(1).maneuvers{trims(leaf), i}.dyaw, 4);
                angle = wrapToPi(round(angles(leaf) + displacement, 4));
                if(~ismember(angle, angles))
                    angles = [angles, angle];
                    trims = [trims, i];
                    leafs = [leafs, length(trims)];
                end
            end
            leafs(leafs == leaf) = [];
        end
    end
end

