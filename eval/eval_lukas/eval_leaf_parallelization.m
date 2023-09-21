time_used = cellfun(@(x) x.time_used, data);
memory_usage = cellfun(@(x) x.used_memory, data);

fid = fopen(strcat("leaf_parallelization", '.txt'), 'a+');
fprintf(fid, 'threads: %d, max time: %d, max memory: %d\n', [varargin{1}, max(time_used), max(memory_usage)]);
fclose(fid);