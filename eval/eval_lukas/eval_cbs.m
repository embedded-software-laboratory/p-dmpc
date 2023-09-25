time_used = cellfun(@(x) x.time_used, data);
memory_usage = cellfun(@(x) x.used_memory, data);
branches = cellfun(@(x) x.branches, data);
branches_feasible = cellfun(@(x) x.branches_feasible, data);

fid = fopen(strcat("cbs", '.txt'), 'a+');
fprintf(fid, '%s:: ', scenario_version);
fprintf(fid, 'max time: %d, max memory: %d, max_branches: %d\n', [max(time_used), max(memory_usage), max(branches)]);
fprintf(fid, 'runtimes: [');
fprintf(fid, '%g ', time_used);
fprintf(fid, ']\nbranches: [');
fprintf(fid, '%g ', branches);
fprintf(fid, ']\nfeasible: [');
fprintf(fid, '%g ', branches_feasible);
fprintf(fid, ']\n');
fclose(fid);