function plot_mpa_local_reachable_sets(mpa, options)

    arguments
        mpa (1, 1) MotionPrimitiveAutomaton;
        options.is_allow_non_convex logical = false;
        options.do_export (1, 1) logical = false;
        options.fig (1, 1) matlab.ui.Figure = figure(Visible = "on");
    end

    mpa.plot_local_reachable_sets(fig = options.fig);

    %export figure
    if options.do_export
        options.fig.Visible = "off";
        set(options.fig, 'Units', 'Inches');
        pos = get(options.fig, 'Position');
        set(options.fig, 'PaperPositionMode', 'Auto', 'PaperUnits', 'Inches', 'PaperSize', [pos(3), pos(4)])

        [file_path, ~, ~] = fileparts(mfilename('fullpath')); % get the path of the current file
        idcs = strfind(file_path, filesep); % find all positions of '/'
        one_folder_up = file_path(1:idcs(end) - 1); % one folder up
        folder_target = [one_folder_up, filesep, 'motion_primitive_automaton', filesep, 'local_reachable_sets'];

        if ~isfolder(folder_target)
            % create target folder if not exist
            mkdir(folder_target)
        end

        file_name = ['trims', num2str(n_trims), '_Hp', num2str(Hp)];

        if options.is_allow_non_convex
            file_name = [file_name, '_non-convex'];
        end

        full_path = [folder_target, filesep, file_name];

        if isfile(full_path)
            warning('The file for visualization of the offline reachable sets was already saved.');
        else
            print(options.fig, full_path, '-dpdf', '-r0');
        end

    end

end
