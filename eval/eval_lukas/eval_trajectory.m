time_used = cellfun(@(x) x.time_used, data);
memory_usage = cellfun(@(x) x.used_memory, data);
gs = cellfun(@(x) x.g, data);


fid = fopen(strcat("trajectory", '.txt'), 'a+');
fprintf(fid, '%s:: ', scenario_version);
fprintf(fid, '%s:: ', eval_option);
fprintf(fid, 'experiments: %d, max time: %d, max memory: %d, g: %d\n', [varargin{1}, max(time_used), max(memory_usage), gs(1)]);
fclose(fid);