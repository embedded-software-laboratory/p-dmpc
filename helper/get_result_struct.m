% MIT License
% 
% Copyright (c) 2021 Lehrstuhl Informatik 11 - RWTH Aachen University
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% 
% This file is part of receding-horizon-graph-search.
% 
% Author: i11 - Embedded Software, RWTH Aachen University

function result = get_result_struct(scenario)
    result = struct;
    
    result.scenario = scenario;
    result.controller_outputs = cell(0,1);
    result.iteration_structs = cell(0,1);
    result.vehicle_path_fullres = cell(scenario.nVeh,0);
    result.trajectory_predictions = cell(scenario.nVeh,0);
    result.controller_runtime = zeros(0,1);
    result.step_time = zeros(0,1);
    result.n_expanded = zeros(0,1);
    
    % create output directory
    directory_name = strrep(strcat(result.scenario.name, '_', result.scenario.controller_name),' ','_');
    output_path = fullfile('results', directory_name);
    result.output_path = output_path;
    if ~isfolder(output_path)
        mkdir(output_path);
    end
end

