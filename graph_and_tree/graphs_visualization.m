function graphs_visualization(belonging_vector, edge_weights, varargin)
% GRAPHS_VISUALIZATION Visualize the graph partitioning results
% 
% Input: 
%   
%   belonging_vector: a vector contains different integers. Vertices with
%   the same interger belong to the same subgraph.
%   
%   edge_weights: edge-weights of the graph.
% 
    
    [belonging_vector, edge_weights, ShowWeights, ShowCutEdges] = parse_inputs(belonging_vector, edge_weights, varargin{:});

    % create MATLAB graph object
    if issymmetric(edge_weights)
        % undirected graph if the edge-weights matrix is symmetric
        G = graph(edge_weights);
    else
        % otherwise directed graph
        G = digraph(edge_weights);
    end

    % number of graphs
    graph_indices = unique(belonging_vector); 
    n_graphs = length(graph_indices); 

    % if the number of graphs does not exceed a certain number (for example, 10), we use fixed colors to
    % plot subgraphs to get a better consistency
    threshold_graphs = 6;
    if n_graphs <= threshold_graphs
        CM = jet(threshold_graphs); % colormap
    else
        % otherwise use changable colors 
        CM = jet(n_graphs);
    end

    nVertices = length(belonging_vector);

    % create different colors for different subgraphs
    
    colors = zeros(nVertices,3);
    for i_vertices=1:nVertices
        colors(i_vertices,:) = CM(belonging_vector(i_vertices),:);
    end

    LineStyle = cell(1,size(G.Edges.EndNodes,1));
    EdgeColor = repmat([0 0 0],size(G.Edges.EndNodes,1),1); % default edge color
    for iE=1:size(G.Edges.EndNodes,1)
        if ~ShowCutEdges
            LineStyle{iE} = '-';
        else
            if belonging_vector(G.Edges.EndNodes(iE,1)) == belonging_vector(G.Edges.EndNodes(iE,2))
                % coupling inside group in solid line
                LineStyle{iE} = '-';
            else
                % coupling corss group in dashed line
                LineStyle{iE} = '--';
                EdgeColor(iE,:) = [1 0 0]; % red for cut edges
            end
        end
    end

    % plot
    if ShowWeights
        plot(G,'LineStyle',LineStyle,'Layout','layered','MarkerSize',8,'NodeColor','k','EdgeLabel',round(G.Edges.Weight,2),'EdgeColor',EdgeColor,'NodeFontSize',12,'EdgeFontSize',10,'LineWidth',1,'ArrowSize',10)
    else
        plot(G,'LineStyle',LineStyle,'Layout','layered','MarkerSize',8,'NodeColor','k','EdgeColor',EdgeColor,'NodeFontSize',14,'EdgeFontSize',10)
    end
    xticks('')
    yticks('')
end

%% local function
function [belonging_vector, edge_weights, ShowWeights, ShowCutEdges] = parse_inputs(belonging_vector, edge_weights, varargin)
    % Process optional input and Name-Value pair options
 
    default_value = false; % default not show weights in the graph

    p = inputParser;
    addRequired(p,'belonging_vector',@(x) isnumeric(x) && isvector(x)); % must be numerical matrix
    addRequired(p,'edge_weights',@(x) isnumeric(x) && ismatrix(x)); % must be numerical matrix
    addParameter(p,'ShowWeights',default_value, @(x) islogical(x));
    addParameter(p,'ShowCutEdges',default_value, @(x) islogical(x));
    parse(p, belonging_vector, edge_weights, varargin{:}); % start parsing
    
    % get parsed inputs
    belonging_vector = p.Results.belonging_vector;
    edge_weights = p.Results.edge_weights;
    ShowWeights = p.Results.ShowWeights;
    ShowCutEdges = p.Results.ShowCutEdges;

end
