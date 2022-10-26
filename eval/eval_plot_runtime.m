function eval_plot_runtime(res)
% EVAL_PLOT_RUNTIME Evaluate the runtime of the experiment before deadlock

plot_runtime_data = compute_plot_runtime_data(res); 
plot_runtime(plot_runtime_data);
end