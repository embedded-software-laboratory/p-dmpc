function [referencePath] = modify_lane_changes(lane_change_indices, stepSize, path)

    referencePath = path;
    maxStepSize = max(stepSize);

    for i = 1:length(lane_change_indices)
        if lane_change_indices(i) ~= 0
            middleIndexLaneBeforeChange = lane_change_indices(i,1) +  uint8((lane_change_indices(i,2) - lane_change_indices(i,1)) / 2);
            middleIndexChangeLane = lane_change_indices(i,3) + uint8((lane_change_indices(i,4) - lane_change_indices(i,3)) / 2);
            middlePointLaneBeforeChange = referencePath(middleIndexLaneBeforeChange,:);
            middlePointChangeLane = referencePath(middleIndexChangeLane,:);
            
            for j = 0:((middleIndexChangeLane-middleIndexLaneBeforeChange)-1)
                referencePath((middleIndexLaneBeforeChange+j),:) = referencePath((middleIndexLaneBeforeChange+j),:) + maxStepSize*normalize(middlePointLaneBeforeChange-middlePointChangeLane);
            end

            lane_change_indices(i,:,:,:) = zeros(1,4);
        end
    end

end