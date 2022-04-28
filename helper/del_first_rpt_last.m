function [data_out] = del_first_rpt_last(data_in)
%UNTITLED Delete the first column and repeat the last column
% Example:
%      data_in = [1 2 3;
%                 4 5 6]
%     data_out = [2 3 3;
%                 5 6 6]

    type = class(data_in);
    switch type
        case 'cell' % cell
            data_out = {data_in{:,2:end} data_in{:,end}};
        otherwise % array
            data_out = [data_in(:,2:end) data_in(:,end)];
    end
end