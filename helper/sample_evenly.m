function [array_sampled] = sample_evenly(array,n_sample,dim)
%SAMPLE_EVENLY Sample a array evenly.
%   Input:
%      array: data to be sampled
%      n_sample: number of sample points
%      dim: integer value, either 1 or 2;
%         dim = 1: sample rows (default)
%         dim = 2: sample columns
%   Output:
%       array_sampled: sampled data

    if nargin==2
        dim = 1; % default dim = 1
    end
    
    [m,n] = size(array);
    if dim==1
        indices = round(linspace(1,m,n_sample));
        array_sampled = array(indices,:);
    else
        indices = round(linspace(1,n,n_sample));
        array_sampled = array(:,indices);
    end

end