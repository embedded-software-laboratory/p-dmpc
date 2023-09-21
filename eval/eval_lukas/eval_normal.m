time_used = cellfun(@(x) x.time_used, data);
memory_usage = cellfun(@(x) x.used_memory, data);
n_expanded = cellfun(@(x) x.n_expanded, data);

fid = fopen(strcat("normal", '.txt'), 'a+');
fprintf(fid, 'max time: %d, max memory: %d, n_expanded_max: %d, n_expanded_min: %d\n', [max(time_used), max(memory_usage), max(n_expanded), min(n_expanded)]);
fclose(fid);