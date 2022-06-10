function plot_coupling_lines(M, x0, varargin)
% PLOT_COUPLING_LINES This function visualizes the coupling between
% each coupled pair. Red lines for coupling of two vehicles inside the same
% group, while blue dashed lines are used if they are in the different
% groups.
% 
% INPUT:
%   M: directed adjacency matrix or edge-weights matrix
% 
%   x0: states
% 
%   belonging_vector (optional input, use [] as input if not fit): a column
%   vector whose value indicate which group each vehicle belongs to.For
%   example,"belonging_vector = [1;2;2;1;3]" means the 1st subgraph =
%   {1,4}, the 2nd subgraph = {2,3} and the 3rd subgraph = {5} 
% 
%   ShowWeights: logical, is show value of weights
% 
    
    % Process optional input and Name-Value pair options
    [M, x0, belonging_vector, ShowWeights] = parse_inputs(M, x0, varargin{:});
    
    nVeh = length(M);

    for v = 1:nVeh
        x = x0(v,:);
        % plot directed coupling
        adjacent_vehicles = find(M(v,:) ~= 0);

        for adj_v = adjacent_vehicles
            adj_x = x0(adj_v,:);

            % plot adjacency
            MaxHeadSize = 0.3/norm([adj_x(1)-x(1),adj_x(2)-x(2)]); % to keep the arrow size 
            if ~isempty(belonging_vector) && belonging_vector(v) ~= belonging_vector(adj_v)
                % couplings inside a group will be shown in red lines, while couplings between
                % groups will be shown in blue dashed lines.
                quiver(x(1),x(2),adj_x(1)-x(1),adj_x(2)-x(2),'AutoScale','off','LineWidth',1,'Color','b','LineStyle','--','MaxHeadSize',MaxHeadSize)
            else
                quiver(x(1),x(2),adj_x(1)-x(1),adj_x(2)-x(2),'AutoScale','off','LineWidth',1,'Color','b','LineStyle',':','MaxHeadSize',MaxHeadSize)
            end

            if ShowWeights
                % plot coupling weights
                text((x(1)+adj_x(1))/2,(x(2)+adj_x(2))/2,...
                    num2str(round(M(v,adj_v),2)),'FontSize', 12, 'LineWidth',1,'Color','b');
            end
        end
    end
end

%% local function
function [M, x0, belonging_vector, ShowWeights] = parse_inputs(M, x0, varargin)
    % Process optional input and Name-Value pair options
    
    default_belonging_vector = ones(1,length(M)); % default all vehicles are in the same group
    default_ShowWeights = false;

    p = inputParser;
    addRequired(p,'M',@(x) ismatrix(x) && (isnumeric(x) || islogical(x))); % must be numerical matrix
    addRequired(p,'x0',@(x) isstruct(x) || ismatrix(x) && (isnumeric(x))); % must be numerical matrix
    addOptional(p,'belonging_vector', default_belonging_vector, @(x) (isnumeric(x) && isvector(x)) || isempty(x)); % must be numerical vector or empty
    addParameter(p,'ShowWeights',default_ShowWeights, @(x) islogical(x));

    parse(p, M, x0, varargin{:}); % start parsing
    
    % get parsed inputs
    M = p.Results.M;
    x0 = p.Results.x0;
    belonging_vector = p.Results.belonging_vector;
    ShowWeights = p.Results.ShowWeights;

end