% If we get the following error:
% "Could not create an SPMD block to match the requested size: 7. The parallel pool size is: 8,
% the active SPMD context is of size: 8."
% That means that we have leftover Composite objects in our workspace.
% E.g. if we started one run with 6 vehicles and have a 1x6 Composite ans
% in our workspace, we will get this error when running 5 or 7 vehicles
% next.
% Using 'clear' to clear the workspace solves this.
function get_parallel_pool(no_of_workers)
    %STARTPOOL Start a pool with no_of_vehicles workers unless it already exists
    % This is better than relying on MATLAB to automatically start a pool,
    % because MATLAB will create as many workers as there are cores on the
    % system - even if there is not enough RAM for that, leading to a crash.
    % As a personal recommendation I wouldn't create more than 2-3 threads on an
    % average laptop.
    old_pool = gcp('nocreate');

    if ~any(size(old_pool)) % Check if gcp('nocreate') returning anything at all
        disp('No pool found, creating new pool');
        create_parpool(no_of_workers);
    elseif old_pool.NumWorkers ~= no_of_workers % If gcp('nocreate') returned something, check the number of workers
        fprintf('Found old pool with %i workers, but we need a pool with %i workers\n', ...
            old_pool.NumWorkers, ...
            no_of_workers);

        delete(old_pool);
        create_parpool(no_of_workers);
    else % Else, everything is fine and we don't need to do anything
        fprintf('Parallel pool with at least %i workers (we have %i workers) already exists, not creating a new one\n', ...
            no_of_workers, ...
            old_pool.NumWorkers);
    end

end

function create_parpool(no_of_workers)
    parpool('local', no_of_workers);
end
