function motionGraphList = create_motion_graph_list(trim_set, vehicle_amount)
    model = BicycleModel(2.2,2.2);
    primitive_dt = 1;
    load(trim_set);

    motionGraphList = [];
    for i = 1:vehicle_amount
        motionGraph = MotionGraph(model, u_trims, trim_adjacency, primitive_dt);
        motionGraphList = [motionGraphList, motionGraph];
    end
end

