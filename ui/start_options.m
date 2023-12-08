function [labOptions] = start_options()
    disp('App is executed, select your configuration there')
    %% Create UI and populate
    ui = CPMStartOptionsUI();

    % Simulation
    % scenario
    scenario = list_scenario(ui);
    ui.ScenarioDropDown.Items = scenario(:, 2);
    ui.ScenarioDropDown.Value = scenario(1, 2);

    % controlStrategy
    controlStrategy = list_control_strategy();
    ui.ControlStrategyListBox.Items = controlStrategy(:, 2);
    ui.ControlStrategyListBox.ValueChangedFcn = @(~, ~) callbackPBSelected(ui);

    % Coupler
    default_coupler = CouplingStrategies.reachable_set_coupling;
    [~, coupling_strategies] = enumeration(default_coupler);
    ui.CouplerDropDown.Items = coupling_strategies;
    ui.CouplerDropDown.Value = default_coupler;

    % priorityAssignmentMethod
    priorityAssignmentMethod = list_priority_assignment_methods();
    ui.PrioritizerDropDown.Items = priorityAssignmentMethod(:, 2);

    % Weigher
    default_weigher = WeightStrategies.distance_weight;
    [~, weight_strategies] = enumeration(default_weigher);
    ui.WeigherDropDown.Items = weight_strategies;
    ui.WeigherDropDown.Value = default_weigher;

    % Cutter
    default_cutter = CutStrategies.iterative_min_cut;
    [~, cut_strategies] = enumeration(default_cutter);
    ui.CutterDropDown.Items = cut_strategies;
    ui.CutterDropDown.Value = default_cutter;

    % Optimizer
    list_optimizer = list_optimizer_prioritized();
    ui.OptimizerDropDown.Items = list_optimizer(:, 2);
    ui.OptimizerDropDown.Value = list_optimizer(1, 2);

    % Constraints from successor (lower-priority vehicles)
    constraint_from_successor = list_constraint_from_successor();
    ui.ConstraintFromSuccessorDropDown.Items = constraint_from_successor(:, 2);
    ui.ConstraintFromSuccessorDropDown.Value = constraint_from_successor(2, 2);

    % Constraints for successors at the lanelet crossing area
    ui.ConstraintsforsuccessorsatthelaneletcrossingareaDropDown.Items = ...
        { ...
         'None'; ...
         'Intersecting lanelets'; ...
         'Intersecting and merging lanelets'; ...
         'Intersecting and merging lanelets at the intersection'; ...
     };
    ui.ConstraintsforsuccessorsatthelaneletcrossingareaDropDown.Value = 'None';
    ui.ConstraintsforsuccessorsatthelaneletcrossingareaDropDown.ItemsData = {1, 2, 4, 3};

    % compute_in_parallel
    compute_in_parallel = list_is_parl();
    ui.ComputationmodeListBox.Items = compute_in_parallel(:, 2);

    % change visibility of vehicle ID selection depending on environment selection
    %ui.ControlStrategyListBox.ValueChangedFcn = @(~, ~) setVisualizationVisibility(ui);
    ui.EnvironmentButtonGroup.SelectionChangedFcn = @(~, ~) setEnvironmentElementsVisibility(ui);

    %% load previous choices, if possible
    try %#ok<TRYNC>
        previousSelection = load([tempdir 'scenarioControllerSelection']);
        ui.EnvironmentButtonGroup.SelectedObject = ui.EnvironmentButtonGroup.Buttons(previousSelection.environmentSelection);
        ui.ComputationmodeListBox.Value = previousSelection.isParlSelection;
        ui.CustomReferencePathsEditField.Value = previousSelection.path_ids;

        ui.ScenarioDropDown.Value = previousSelection.scenarioSelection;
        ui.ControlStrategyListBox.Value = previousSelection.controlStrategySelection;

        ui.CouplerDropDown.Value = previousSelection.coupling_strategy;
        ui.PrioritizerDropDown.Value = previousSelection.priorityAssignmentMethodSelection;
        ui.WeigherDropDown.Value = previousSelection.weight_strategy;
        ui.CutterDropDown.Value = previousSelection.cut_strategy;
        ui.NumberofvehiclesEditField.Value = previousSelection.vehicleAmountSelection;
        ui.VisualizeListBox.Value = previousSelection.visualizationSelection;

        % sample time [s]
        ui.SampleTimesSpinner.Value = previousSelection.dtSelection;

        % predicion horizon
        ui.PredictionHorizonSpinner.Value = previousSelection.HpSelection;

        % MPA type
        ui.MPADropDown.Value = previousSelection.mpa_typeSelection;

        % simulation duration [s]
        ui.DurationsEditField.Value = previousSelection.T_endSelection;

        % maximum allowed number of computation levels
        ui.MaxComputationLevelsSpinner.Value = previousSelection.max_num_CLsSelection;

        % Stategy to let vehicle with the right-of-way consider vehicle without the right-of-way
        ui.ConstraintFromSuccessorDropDown.Value = previousSelection.constraint_from_successorSelection;

        % Strategy to let vehicle without the right-of-way enter the crossing area of its lanelet with lanelet of its coupled vehicle
        ui.ConstraintsforsuccessorsatthelaneletcrossingareaDropDown.Value = previousSelection.strategy_enter_crossing_areaSelection;

        callbackPBSelected(ui);

        % Which optimizer to use
        ui.OptimizerDropDown.Value = previousSelection.optimizer;
    end

    %% Trigger UI change handles
    setEnvironmentElementsVisibility(ui);

    %% Run App
    % wait until UI is closed or Start pressed
    %   emulating `waitfor(ui)`
    while true

        if ~isvalid(ui) % if app close
            error('App closed, press start button for experiment')
        elseif ui.StartButton.Value % start button pressed
            break
        end

        pause(0.1)
    end

    disp('Configuration Setup App finished, extracting data')

    %% Extract Choices & Save for next time
    environmentSelection = get_environment_selection(ui);

    scenarioSelection = ui.ScenarioDropDown.Value;
    controlStrategySelection = ui.ControlStrategyListBox.Value;
    coupling_strategy = ui.CouplerDropDown.Value;
    priorityAssignmentMethodSelection = ui.PrioritizerDropDown.Value;
    weight_strategy = ui.WeigherDropDown.Value;
    cut_strategy = ui.CutterDropDown.Value;
    vehicleAmountSelection = ui.NumberofvehiclesEditField.Value;
    visualizationSelection = ui.VisualizeListBox.Value;
    isParlSelection = ui.ComputationmodeListBox.Value;
    path_ids = ui.CustomReferencePathsEditField.Value;
    optimizer = ui.OptimizerDropDown.Value;

    % sample time [s]
    dtSelection = ui.SampleTimesSpinner.Value;
    % predicion horizon
    HpSelection = ui.PredictionHorizonSpinner.Value;
    % MPA type
    mpa_typeSelection = ui.MPADropDown.Value;
    % simulation duration [s]
    T_endSelection = ui.DurationsEditField.Value;
    % maximum allowed number of computation levels
    max_num_CLsSelection = ui.MaxComputationLevelsSpinner.Value;
    % Stategy to let vehicle with the right-of-way consider vehicle without the right-of-way
    constraint_from_successorSelection = ui.ConstraintFromSuccessorDropDown.Value;
    % Strategy to let vehicle without the right-of-way enter the crossing area of its lanelet with lanelet of its coupled vehicle
    strategy_enter_crossing_areaSelection = ui.ConstraintsforsuccessorsatthelaneletcrossingareaDropDown.Value;

    save([tempdir 'scenarioControllerSelection'], ...
        'optimizer', ...
        'environmentSelection', ...
        'scenarioSelection', ...
        'controlStrategySelection', ...
        'priorityAssignmentMethodSelection', ...
        'vehicleAmountSelection', ...
        'visualizationSelection', ...
        'isParlSelection', ...
        'dtSelection', ...
        'HpSelection', ...
        'mpa_typeSelection', ...
        'T_endSelection', ...
        'max_num_CLsSelection', ...
        'path_ids', ...
        'constraint_from_successorSelection', ...
        'strategy_enter_crossing_areaSelection' ...
    );

    %% Convert to legacy/outputs
    % initialize
    labOptions = Config();

    labOptions.environment = get_environment_selection(ui, true);

    controlStrategyHelper = controlStrategy{ ...
                                                strcmp(controlStrategy(:, 2), controlStrategySelection), ...
                                                2};

    labOptions.is_prioritized = (strcmp(controlStrategyHelper, 'pb non-coop'));

    labOptions.optimizer_type = OptimizerType(string(optimizer));

    labOptions.amount = vehicleAmountSelection;

    path_ids = ui.CustomReferencePathsEditField.Value;

    if path_ids ~= ""
        path_ids(~isstrprop(path_ids, 'digit')) = ' '; %replace non-numeric characters with empty space
        labOptions.path_ids = str2double(strsplit(strtrim(path_ids)));
    else
        labOptions.path_ids = [];
    end

    labOptions.options_plot_online = OptionsPlotOnline();
    labOptions.options_plot_online.is_active = strcmp(visualizationSelection, 'yes');

    isParlHelper = compute_in_parallel{ ...
                                           strcmp(compute_in_parallel(:, 2), isParlSelection), ...
                                           2};

    labOptions.compute_in_parallel = strcmp(isParlHelper, 'thread parallel') ...
        || strcmp(isParlHelper, 'physically parallel');

    % hacky way to get nuc simulation
    if strcmp(isParlHelper, 'physically parallel') ...
            && labOptions.environment == Environment.Simulation
        labOptions.environment = Environment.SimulationDistributed;
    end

    scenario = list_scenario(ui); % Update scenario since the selected options may differ now
    labOptions.scenario_type = scenario{ ...
                                            strcmp(scenario(:, 2), scenarioSelection), ...
                                            1};

    labOptions.priority = priorityAssignmentMethod{ ...
                                                       strcmp(priorityAssignmentMethod(:, 2), priorityAssignmentMethodSelection), ...
                                                       1};

    labOptions.coupling = CouplingStrategies(coupling_strategy);
    labOptions.weight = WeightStrategies(weight_strategy);
    labOptions.cut = CutStrategies(cut_strategy);

    % sample time [s]
    labOptions.dt_seconds = dtSelection;

    % predicion horizon
    labOptions.Hp = HpSelection;

    % MPA type
    labOptions.mpa_type = mpa_typeSelection;

    % simulation duration [s]
    labOptions.T_end = T_endSelection;

    % maximum allowed number of computation levels
    labOptions.max_num_CLs = max_num_CLsSelection;

    % Stategy to let vehicle with the right-of-way consider vehicle without the right-of-way
    labOptions.constraint_from_successor = constraint_from_successor{ ...
                                                                         strcmp(constraint_from_successor(:, 2), constraint_from_successorSelection), ...
                                                                         1 ...
                                                                     };

    % Strategy to let vehicle without the right-of-way enter the crossing area of its lanelet with lanelet of its coupled vehicle
    labOptions.strategy_enter_lanelet_crossing_area = strategy_enter_crossing_areaSelection;

    % Validate Config file
    labOptions = labOptions.validate();

    % Write Config to disk
    encodedJSON = jsonencode(labOptions);
    fid = fopen('Config.json', 'w');
    fprintf(fid, encodedJSON);
    fclose('all');
    % save('config.mat','labOptions');

    % close app
    ui.delete;
end

function out = get_environment_selection(ui, output_as_enum)
    % selection of environment
    out = ui.EnvironmentButtonGroup.SelectedObject == ui.EnvironmentButtonGroup.Buttons;

    % is CPM lab selected
    if nargin > 1 && output_as_enum

        if isequal([1 0 0], out)
            out = Environment.CpmLab;
        elseif isequal([0 1 0], out)
            out = Environment.Simulation;
        elseif isequal([0 0 1], out)
            out = Environment.UnifiedLabApi;
        end

    end

end

function setEnvironmentElementsVisibility(ui)
    % if lab mode is selected
    is_lab_selection = (get_environment_selection(ui, true) == Environment.CpmLab ...
        || get_environment_selection(ui, true) == Environment.UnifiedLabApi);

    % sample time is automatically set in the lab
    ui.SampleTimesSpinner.Enable = ~is_lab_selection;

    scenario = list_scenario(ui);
    ui.ScenarioDropDown.Items = scenario(:, 2);
end

% callback function if parallel computation is selected/unselected
function callbackPBSelected(ui)

    if strcmp(ui.ControlStrategyListBox.Value, 'pb non-coop')
        ui.MaxComputationLevelsSpinner.Enable = 'on';
        ui.ConstraintFromSuccessorDropDown.Enable = 'on';
        ui.ConstraintsforsuccessorsatthelaneletcrossingareaDropDown.Enable = 'on';
        ui.ComputationmodeListBox.Enable = 'on';

        ui.Label_4.Visible = 'Off';

        list_optimizer = list_optimizer_prioritized();
        ui.OptimizerDropDown.Items = list_optimizer(:, 2);
        ui.OptimizerDropDown.Value = list_optimizer(1, 2);
    else
        ui.MaxComputationLevelsSpinner.Enable = 'off';
        ui.ConstraintFromSuccessorDropDown.Enable = 'off';
        ui.ConstraintsforsuccessorsatthelaneletcrossingareaDropDown.Enable = 'off';

        % centralized can never be run in parallel
        ui.ComputationmodeListBox.Enable = 'off';
        ui.ComputationmodeListBox.Value = 'no';

        ui.Label_4.Text = sprintf("If not priority-based, the maximum allowed number of computation levels is irrelevant.");
        ui.Label_4.Visible = 'On';

        list_optimizer = list_optimizer_centralized();
        ui.OptimizerDropDown.Items = list_optimizer(:, 2);
        ui.OptimizerDropDown.Value = list_optimizer(1, 2);
    end

end

function [list] = list_is_parl
    list = { ...
                '1', 'sequential, logically parallel'; ...
                '2', 'thread parallel'; ...
                '3', 'physically parallel'; ...
            };
end

function [list] = list_scenario(ui)
    is_unified_lab_interface_selected = get_environment_selection(ui, true) == Environment.UnifiedLabApi;

    if is_unified_lab_interface_selected % Only allow lanelet2 maps here
        list = { ...
                    'Lanelet2', 'Lanelet2'; ...
                    'Lab_default', 'Lab Default (ULA only)' ...
                };
    else
        list = { ...
                    ScenarioType.circle, 'Circle'; ...
                    ScenarioType.commonroad, 'Commonroad'; ...
                    ScenarioType.lanelet2, 'Lanelet2'; ...
                    ScenarioType.lab_default, 'Lab Default' ...
                };
    end

end

function [list] = list_control_strategy
    list = { ...
                '1', 'centralized'; ...
                '2', 'pb non-coop'; ...
            };
end

function [list] = list_optimizer_centralized
    list = {
            '1', 'MatlabOptimal';
            '2', 'CppOptimal';
            '3', 'CppSampled';
            };
end

function [list] = list_optimizer_prioritized
    list = {
            '1', 'MatlabOptimal';
            '2', 'MatlabSampled';
            '3', 'CppOptimal';
            };
end

function [list] = list_priority_assignment_methods
    list = { ...
                PriorityStrategies.coloring_priority, 'Coloring Priority'; ...
                PriorityStrategies.constant_priority, 'Constant Priority'; ...
                PriorityStrategies.random_priority, 'Random Priority'; ...
                PriorityStrategies.FCA_priority, 'FCA Priority'; ...
                PriorityStrategies.STAC_priority, 'STAC Priority'
            };
end

function [list] = list_constraint_from_successor
    list = { ...
                ConstraintFromSuccessor.none, 'None'; ...
                ConstraintFromSuccessor.area_of_standstill, 'Area of standstill'; ...
                ConstraintFromSuccessor.area_of_previous_trajectory, 'Area of previous trajectory'
            };
end
