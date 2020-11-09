function reference = create_reference(init_poses, target_poses, trim_indices, motion_graph)
%CREATE_REFERENCE Summary of this function goes here
%   Detailed explanation goes here
maneuver = motion_graph.motionGraphList(1).maneuvers{1, 1};

assert(motion_graph.motionGraphList(1).trims(1).steering == 0)

xs = 

reference = num2cell(poses.xs);
reference.ts = 

end

