function compile
    % compiles graph search with cmake
    % For Debugging: Set cmake debug or user mex -g COMPFLAGS='$COMPFLAGS -std:c++20' -Ihlc\optimizer\graph_search_mex\include -I"{link to boost root}" -L"{link to boost lib}" hlc\optimizer\graph_search_mex\src\main.cpp -outdir hlc/optimizer/graph_search_mex/ -output optimizer
    folder = fileparts(mfilename('fullpath'));
    is_mex_compiled = isequal(fileparts(which('optimizer_mex')), folder);
    if ~is_mex_compiled
        cwd = cd(folder);
        cleanup_obj = onCleanup(@() cd(cwd));
        fprintf('Compiling C++ graph search\n');
        if ispc
            cmake_path = fullfile(matlabroot, 'bin', computer('arch'), 'cmake', 'bin', 'cmake.exe');
            cmake_command = ['"' cmake_path '"' ' .'];
            cmake_build_command = ['"' cmake_path '"' ' --build .'];
        else
            %cmake_path = fullfile(matlabroot, 'bin', computer('arch'), 'cmake', 'bin', 'cmake');
            cmake_command = ['cmake'  ' .'];
            cmake_build_command = ['cmake'  ' --build .'];
        end
        [status1, cmdout1] = system(cmake_command);
        [status2, cmdout2] = system(cmake_build_command);
        if status2 ~= 0
            display(cmdout1);
            display(cmdout2);
            error("CMake Error");
        else
            display("Graph Search Compiled");
        end
    end
end
