function plot_mpa(scenario,options)
arguments
    scenario            (1,1) Scenario;
    options.y_lim       (1,2) double = [-0.1, 1.6];
end
    mpa = scenario.mpa;
    
    trim_inputs = mpa.trims;
    trim_adjacency = mpa.transition_matrix_single(:,:,1);
    
    fig = figure("Visible","off");
    angle = rad2deg([trim_inputs.steering]);
    speed = [trim_inputs.speed];
    G = digraph(trim_adjacency,'omitSelfLoops');
    
    plot(G,'XData',angle,'YData',speed);

    xlabel('Steering Angle $\delta$ [$\circ$]','Interpreter','tex');
    ylabel('Speed $\nu$ [m/s]','Interpreter','tex');
    
    xlim([-37,37])
    ylim(options.y_lim)
    grid on

    file_ext = '.pdf';
    
    folder_path = FileNameConstructor.gen_results_folder_path(scenario.options);
    [~,file_name,~] = fileparts(FileNameConstructor.get_mpa_name(scenario.options));
    filepath = fullfile(folder_path,[file_name file_ext]);
    set_figure_properties(fig,'document')
    export_fig(fig, filepath)
    close(fig);
end