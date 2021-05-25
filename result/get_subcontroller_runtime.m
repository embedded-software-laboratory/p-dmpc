function [ subcontroller_runtime ] = get_subcontroller_runtime(info,scenario)

    if numel(info) == 1
        subcontroller_runtime = info.subcontroller_runtime*ones(scenario.nVeh,1);
        return
    end
    subcontroller_runtime = zeros(scenario.nVeh,1);
    for i = 1:scenario.nVeh
        subcontroller_runtime(i) = info{i}.subcontroller_runtime;
    end
    
end
