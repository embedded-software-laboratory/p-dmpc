function compile_graph_search(varargin)
    %COMPILE Compilation of the C++ graph search
    %   compiles both the prioritized Graph Saecrh and the centralized Graph
    %   Search versions and produces the according mex-files.

    default_centralized_recompile = false;
    default_priority_recompile = false;
    default_centralized_compile = true;
    default_priority_compile = true;
    default_cmake_path = 'cmake'; %fullfile(matlabroot,'bin', computer('arch'), 'cmake', 'bin', 'cmake');
    default_compiler_path = '';

    p = inputParser;
    addParameter(p, 'centralized_compile', default_centralized_compile, @islogical);
    addParameter(p, 'priority_compile', default_priority_compile, @islogical);
    addParameter(p, 'centralized_recompile', default_centralized_recompile, @islogical);
    addParameter(p, 'priority_recompile', default_priority_recompile, @islogical);
    addParameter(p, 'cmake_path', default_cmake_path, @(x) validateattributes(x, {'char'}, {'nonempty'}));
    addParameter(p, 'compiler_path', default_compiler_path, @(x) validateattributes(x, {'char'}, {'nonempty'}));

    parse(p, varargin{:});

    centralized_compile = p.Results.centralized_compile;
    priority_compile = p.Results.priority_compile;

    filepath = fileparts(mfilename('fullpath'));
    [~, name, ~] = fileparts(filepath);

    if centralized_compile

        if ~p.Results.centralized_recompile
            filepath_centralized = fullfile(filepath, [name '_centralized_mex.' mexext]);
            centralized_compile = ~isfile(filepath_centralized);
        end

    end

    if priority_compile

        if ~p.Results.priority_recompile
            filepath_priority = fullfile(filepath, [name '_priority_mex.' mexext]);
            priority_compile = ~isfile(filepath_priority);
        end

    end

    if centralized_compile || priority_compile
        compiler = '';

        if ~isempty(p.Results.compiler_path)
            compiler = [' -DCMAKE_CXX_COMPILER=', p.Results.compiler_path, ' '];
        end

        build_path = fullfile(filepath, 'build');

        if ~isfolder(build_path)
            mkdir(build_path);
        end

        %cmake_command = [p.Results.cmake_path, ' -DCMAKE_BUILD_TYPE=Release -DCMAKE_MAKE_PROGRAM=', p.Results.ninja_path, compiler, ' -G Ninja -S ', filepath, ' -B ' build_path]
        cmake_command = [p.Results.cmake_path, ' -DCMAKE_BUILD_TYPE=Release', compiler, ' -S ', filepath, ' -B ' build_path]

        [status, cmdout] = system(cmake_command, "-echo");

        if status ~= 0
            error("CMake Error");
        end

        if centralized_compile
            build_command = [p.Results.cmake_path, ' --build ', build_path, ' --target ', name, '_centralized_mex -j ', num2str(maxNumCompThreads), ' --config Release']

            [status, cmdout] = system(build_command, "-echo");

            if status ~= 0
                error("CMake Error");
            end

        end

        if priority_compile
            build_command = [p.Results.cmake_path, ' --build ', build_path, ' --target ', name, '_priority_mex -j ', num2str(maxNumCompThreads), ' --config Release']

            [status, cmdout] = system(build_command, "-echo");

            if status ~= 0
                error("CMake Error");
            end

        end

    end

end
