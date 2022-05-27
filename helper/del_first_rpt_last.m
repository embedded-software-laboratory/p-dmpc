function [data_out] = del_first_rpt_last(data_in, repeat_times)
% DEL_FIRST_RPT_LAST Delete the first column and repeat the last column
% 
% INPUT:
%   data_in: input data, can be either matrix or cell array
% 
%   repeat_times: how many columns in front should be deleted and how many times
%   that the last column should be repeated. Default to 1.
% 
% OUTPUT:
%   data_out: output data
%   
% Example:
%      data_in = [1 2 3;
%                 4 5 6]
%     data_out = [2 3 3;
%                 5 6 6]

    if nargin == 1
        repeat_times = 1; % default value
    end

    data_out = horzcat(data_in(:,1+repeat_times:end), repmat(data_in(:,end),1,repeat_times));
end