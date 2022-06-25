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
    [M, x0, belonging_vector, coupling_info, ShowWeights] = parse_inputs(M, x0, varargin{:});
    
    % if the given matrix is adjacency matrix but not edge-weights matrix
    if all(M==1|M==0,"all")
        ShowWeights = false;
    end
    
    nVeh = length(M);
    
    color_main = [0 0 0];
    color_minor = [0.5 0.5 0.5];
    for v = 1:nVeh
        x = x0(v,:);
        % plot directed coupling
        adjacent_vehicles = find(M(v,:) ~= 0);

        % plot couplings that are ignored by letting vehicles without
        % right-of-way not enter the lanelet intersecting area
        if ~isempty(coupling_info)
            find_self = [coupling_info.veh_with_ROW] == v;
            find_ignored = [coupling_info.is_ignored] == true;
            find_targrt = all([find_self;find_ignored],1);

            if any(find_targrt)
                all_adjacent_vehicles = coupling_info(find_targrt).veh_without_ROW;
                coupling_ignored = setdiff(all_adjacent_vehicles,adjacent_vehicles);
                for i_ignored=coupling_ignored(:)'
                    ignored_x = x0(i_ignored,:);
                    MaxHeadSize = 0.3/norm([ignored_x(1)-x(1),ignored_x(2)-x(2)]); % to keep the arrow size 
                    % couplings that are ignored will be shown in grey solid lines
                    quiver(x(1),x(2),ignored_x(1)-x(1),ignored_x(2)-x(2),'AutoScale','off','LineWidth',1,'Color',color_minor,'LineStyle','-','MaxHeadSize',MaxHeadSize)
                end
            end
        end

        for adj_v = adjacent_vehicles
            adj_x = x0(adj_v,:);

            % plot adjacency
            MaxHeadSize = 0.3/norm([adj_x(1)-x(1),adj_x(2)-x(2)]); % to keep the arrow size 
            if ~isempty(belonging_vector) && belonging_vector(v) ~= belonging_vector(adj_v)
                % couplings between groups will be shown in black dashed lines.
                quiver(x(1),x(2),adj_x(1)-x(1),adj_x(2)-x(2),'AutoScale','off','LineWidth',1,'Color',color_main,'LineStyle',':','MaxHeadSize',MaxHeadSize)
            else
                % couplings inside a group will be shown in black solid lines
                quiver(x(1),x(2),adj_x(1)-x(1),adj_x(2)-x(2),'AutoScale','off','LineWidth',1,'Color',color_main,'LineStyle','-','MaxHeadSize',MaxHeadSize)
            end

            if ShowWeights
                % plot coupling weights
                text((x(1)+adj_x(1))/2,(x(2)+adj_x(2))/2,...
                    num2str(round(M(v,adj_v),2)),'FontSize', 12, 'LineWidth',1,'Color',color_main);
            end
        end
    end


end

%% local function
function [M, x0, belonging_vector, coupling_info, ShowWeights] = parse_inputs(M, x0, varargin)
    % Process optional input and Name-Value pair options
    
    default_belonging_vector = ones(1,length(M)); % default all vehicles are in the same group
    default_coupling_info = [];
    default_ShowWeights = false;


    p = inputParser;
    addRequired(p,'M',@(x) ismatrix(x) && (isnumeric(x) || islogical(x))); % must be numerical matrix
    addRequired(p,'x0',@(x) isstruct(x) || ismatrix(x) && (isnumeric(x))); % must be numerical matrix
    addOptional(p,'belonging_vector', default_belonging_vector, @(x) (isnumeric(x) && isvector(x)) || isempty(x)); % must be numerical vector or empty
    addOptional(p,'coupling_info', default_coupling_info, @(x) isstruct(x) || isempty(x)); % must be struct or empty
    addParameter(p,'ShowWeights',default_ShowWeights, @(x) islogical(x));

    parse(p, M, x0, varargin{:}); % start parsing
    
    % get parsed inputs
    M = p.Results.M;
    x0 = p.Results.x0;
    belonging_vector = p.Results.belonging_vector;
    coupling_info = p.Results.coupling_info;
    ShowWeights = p.Results.ShowWeights;

end