function [u, y_pred, info] = centralized_controller(scenario, iter)
    [u, y_pred, info] = graph_search(scenario, iter);
end

