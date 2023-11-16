function plot_partitioned_graph(experiment_result, optional)

    arguments
        experiment_result (1, 1) struct
        optional.show_weights (1, 1) logical = false
        optional.show_cut_edges (1, 1) logical = false
        optional.i_step (1, 1) uint16 = size(experiment_result.belonging_vector, 2)
        optional.do_export (1, 1) logical = true;
        optional.fig (1, 1) matlab.ui.Figure = figure(Visible = "on");
    end

    belonging_vector = experiment_result.belonging_vector(:, optional.i_step);
    edge_weights = experiment_result.iteration_structs{optional.i_step}.weighted_coupling_reduced;

    if issymmetric(edge_weights)
        % undirected graph if the edge-weights matrix is symmetric
        G = graph(edge_weights);
    else
        % otherwise directed graph
        G = digraph(edge_weights);
    end

    line_style = cell(1, size(G.Edges.EndNodes, 1));
    edge_color = zeros(size(G.Edges.EndNodes, 1), 3); % default edge color

    for iE = 1:size(G.Edges.EndNodes, 1)
        is_sequential_coupling = ( ...
            belonging_vector(G.Edges.EndNodes(iE, 1)) == belonging_vector(G.Edges.EndNodes(iE, 2)) ...
        );
        is_solid_line = is_sequential_coupling || ~optional.show_cut_edges;

        if is_solid_line
            % coupling inside group in solid line
            line_style{iE} = '-';
        else
            % coupling corss group in dashed line
            line_style{iE} = '--';
            edge_color(iE, :) = rwth_color_order(5); % red for cut edges
        end

    end

    plot_handle = plot(G, ...
        LineStyle = line_style, ...
        Layout = 'layered', ...
        NodeColor = 'k', ...
        EdgeColor = edge_color, ...
        LineWidth = 1, ...
        MarkerSize = 3 ...
    );

    if isa(G, 'digraph')
        plot_handle.ArrowSize = 5;
    end

    if optional.show_weights
        labeledge(plot_handle, 1:numedges(G), round(G.Edges.Weight, 2));
    end

    xticks('')
    yticks('')

    if optional.do_export
        results_folder = FileNameConstructor.gen_results_folder_path(experiment_result.options);
        experiment_prefix = FileNameConstructor.gen_scenario_name(experiment_result.options);
        file_name = [experiment_prefix, sprintf('_partitioned_graph_%d.pdf', optional.i_step)];
        filepath = fullfile(results_folder, file_name);
        set_figure_properties(optional.fig, ExportFigConfig.paper());
        export_fig(optional.fig, filepath);
    end

    if (~optional.fig.Visible); close(optional.fig); end
end
