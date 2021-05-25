function [u, y_pred, info] = centralized_controller(scenario, iter)
    % falsifies controller_runtime slightly
    subcontroller_timer = tic;
    [u, y_pred, info] = graph_search(scenario, iter);
    % falsifies controller_runtime slightly
    info.subcontroller_runtime = toc(subcontroller_timer);
end

