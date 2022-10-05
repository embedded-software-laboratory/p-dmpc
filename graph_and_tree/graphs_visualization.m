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
    
    [belonging_vector, edge_weights, ShowWeights] = parse_inputs(belonging_vector, edge_weights, varargin{:});

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
    for iE=1:size(G.Edges.EndNodes,1)
        if belonging_vector(G.Edges.EndNodes(iE,1)) == belonging_vector(G.Edges.EndNodes(iE,2))
            % coupling inside group in solid line
            LineStyle{iE} = '-';
        else
            % coupling corss group in dashed line
            LineStyle{iE} = '--';
        end
    end

    % plot
%     figure()
    if ShowWeights
        plot(G,'LineStyle',LineStyle,'Layout','layered','MarkerSize',14,'NodeColor','k','EdgeLabel',round(G.Edges.Weight,2),'EdgeColor','k','NodeFontSize',16,'EdgeFontSize',14,'LineWidth',2,'ArrowSize',15)
    else
        plot(G,'LineStyle',LineStyle,'Layout','layered','MarkerSize',14,'NodeColor','k','EdgeColor','k','NodeFontSize',16,'EdgeFontSize',14)
    end
%     title('Results of graph partitioning/merging algorithm')
    
end


%% local function
function [belonging_vector, edge_weights, ShowWeights] = parse_inputs(belonging_vector, edge_weights, varargin)
    % Process optional input and Name-Value pair options
 
    default_value = false; % default not show weights in the graph

    p = inputParser;
    addRequired(p,'belonging_vector',@(x) isnumeric(x) && isvector(x)); % must be numerical matrix
    addRequired(p,'edge_weights',@(x) isnumeric(x) && ismatrix(x)); % must be numerical matrix
    addParameter(p,'ShowWeights',default_value, @(x) islogical(x));
    parse(p, belonging_vector, edge_weights, varargin{:}); % start parsing
    
    % get parsed inputs
    belonging_vector = p.Results.belonging_vector;
    edge_weights = p.Results.edge_weights;
    ShowWeights = p.Results.ShowWeights;

end
