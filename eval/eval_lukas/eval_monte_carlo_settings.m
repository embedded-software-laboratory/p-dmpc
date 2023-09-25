time_used = cellfun(@(x) x.time_used, data);
memory_usage = cellfun(@(x) x.used_memory, data);

fid = fopen(strcat("monte_carlo_settings", '.txt'), 'a+');
fprintf(fid, 'threads: %d, experiments: %d, max time: %d, max memory: %d\n', [varargin{1}, varargin{2}, max(time_used), max(memory_usage)]);
fclose(fid);