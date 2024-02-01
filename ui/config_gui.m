function config = config_gui()
    fprintf('Select your configuration... ')
    %% Create UI and populate
    ui = ConfigGui();

    % Simulation
    % scenario
    scenario = list_scenario(ui);
    ui.ScenarioDropDown.Items = scenario(:, 2);
    ui.ScenarioDropDown.Value = scenario(1, 2);

    % controlStrategy
    controlStrategy = list_control_strategy();
    ui.ControlStrategyListBox.Items = controlStrategy(:, 2);
    ui.ControlStrategyListBox.Value = controlStrategy(1, 2);
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
    default_cutter = CutStrategies.greedy_cut;
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

    % Computation mode
    computation_mode = list_computation_mode();
    ui.ComputationmodeListBox.Items = computation_mode(:, 2);
    ui.ComputationmodeListBox.Value = computation_mode(1, 2);

    % change visibility of vehicle ID selection depending on environment selection
    %ui.ControlStrategyListBox.ValueChangedFcn = @(~, ~) setVisualizationVisibility(ui);
    ui.EnvironmentButtonGroup.SelectionChangedFcn = @(~, ~) setEnvironmentElementsVisibility(ui);

    %% load previous choices, if possible
    try %#ok<TRYNC>
        previousSelection = load([tempdir 'scenarioControllerSelection']);
        ui.EnvironmentButtonGroup.SelectedObject = ui.EnvironmentButtonGroup.Buttons(previousSelection.environmentSelection);
        ui.ComputationmodeListBox.Value = previousSelection.computation_mode_selection;
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

    fprintf('extracting data... ')

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
    computation_mode_selection = ui.ComputationmodeListBox.Value;
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

    save([tempdir 'scenarioControllerSelection'], ...
        'optimizer', ...
        'environmentSelection', ...
        'scenarioSelection', ...
        'controlStrategySelection', ...
        'coupling_strategy', ...
        'priorityAssignmentMethodSelection', ...
        'weight_strategy', ...
        'cut_strategy', ...
        'vehicleAmountSelection', ...
        'visualizationSelection', ...
        'computation_mode_selection', ...
        'dtSelection', ...
        'HpSelection', ...
        'mpa_typeSelection', ...
        'T_endSelection', ...
        'max_num_CLsSelection', ...
        'path_ids', ...
        'constraint_from_successorSelection' ...
    );

    %% Convert to legacy/outputs
    % initialize
    config = Config();

    config.environment = get_environment_selection(ui, true);

    controlStrategyHelper = controlStrategy{ ...
                                                strcmp(controlStrategy(:, 2), controlStrategySelection), ...
                                                2};

    config.is_prioritized = (strcmp(controlStrategyHelper, 'prioritized'));

    config.optimizer_type = OptimizerType(string(optimizer));

    config.amount = vehicleAmountSelection;

    path_ids = ui.CustomReferencePathsEditField.Value;

    if path_ids ~= ""
        path_ids(~isstrprop(path_ids, 'digit')) = ' '; %replace non-numeric characters with empty space
        config.path_ids = str2double(strsplit(strtrim(path_ids)));
    else
        config.path_ids = [];
    end

    config.options_plot_online = OptionsPlotOnline();
    config.options_plot_online.is_active = strcmp(visualizationSelection, 'yes');

    config.computation_mode = computation_mode{ ...
                                                   strcmp(computation_mode(:, 2), computation_mode_selection), ...
                                                   1 ...
                                               };

    scenario = list_scenario(ui); % Update scenario since the selected options may differ now
    config.scenario_type = scenario{ ...
                                        strcmp(scenario(:, 2), scenarioSelection), ...
                                        1};

    config.priority = priorityAssignmentMethod{ ...
                                                   strcmp(priorityAssignmentMethod(:, 2), priorityAssignmentMethodSelection), ...
                                                   1};

    config.coupling = CouplingStrategies(coupling_strategy);
    config.weight = WeightStrategies(weight_strategy);
    config.cut = CutStrategies(cut_strategy);

    % sample time [s]
    config.dt_seconds = dtSelection;

    % predicion horizon
    config.Hp = HpSelection;

    % MPA type
    config.mpa_type = mpa_typeSelection;

    % simulation duration [s]
    config.T_end = T_endSelection;

    % maximum allowed number of computation levels
    config.max_num_CLs = max_num_CLsSelection;

    % Stategy to let vehicle with the right-of-way consider vehicle without the right-of-way
    config.constraint_from_successor = constraint_from_successor{ ...
                                                                     strcmp(constraint_from_successor(:, 2), constraint_from_successorSelection), ...
                                                                     1 ...
                                                                 };

    % Validate Config file
    config = config.validate();

    % close app
    ui.delete;

    fprintf('done.\n')
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

    if strcmp(ui.ControlStrategyListBox.Value, 'prioritized')
        ui.MaxComputationLevelsSpinner.Enable = 'on';
        ui.ConstraintFromSuccessorDropDown.Enable = 'on';
        ui.ComputationmodeListBox.Enable = 'on';

        list_optimizer = list_optimizer_prioritized();
        ui.OptimizerDropDown.Items = list_optimizer(:, 2);
        ui.OptimizerDropDown.Value = list_optimizer(1, 2);
    else
        ui.MaxComputationLevelsSpinner.Enable = 'off';
        ui.ConstraintFromSuccessorDropDown.Enable = 'off';

        % centralized can never be run in parallel
        ui.ComputationmodeListBox.Enable = 'off';
        computation_mode = list_computation_mode();
        ui.ComputationmodeListBox.Value = computation_mode(1, 2);

        list_optimizer = list_optimizer_centralized();
        ui.OptimizerDropDown.Items = list_optimizer(:, 2);
        ui.OptimizerDropDown.Value = list_optimizer(1, 2);
    end

end

function [list] = list_computation_mode
    list = { ...
                ComputationMode.sequential, 'sequential, logically parallel'; ...
                ComputationMode.parallel_threads, 'thread parallel'; ...
                ComputationMode.parallel_physically, 'physically parallel'; ...
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
                '1', 'prioritized'; ...
                '2', 'centralized'; ...
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
                PriorityStrategies.STAC_priority, 'STAC Priority'; ...
                PriorityStrategies.optimal_priority, 'Optimal Priority'; ...
                PriorityStrategies.explorative_priority, 'Explorative Priority'
            };
end

function [list] = list_constraint_from_successor
    list = { ...
                ConstraintFromSuccessor.none, 'None'; ...
                ConstraintFromSuccessor.area_of_standstill, 'Area of standstill'; ...
                ConstraintFromSuccessor.area_of_previous_trajectory, 'Area of previous trajectory'
            };
end
