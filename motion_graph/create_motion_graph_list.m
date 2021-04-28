function [motionGraphList,model] = create_motion_graph_list(trim_set, vehicle_amount, offset, dt)
    model = BicycleModel(2.2,2.2);
    load(trim_set);

    motionGraphList = [];
    for i = 1:vehicle_amount
        motionGraph = MotionGraph(model, u_trims, trim_adjacency, offset, dt);
        motionGraphList = [motionGraphList, motionGraph];
    end
end