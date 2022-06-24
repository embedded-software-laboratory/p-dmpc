function [labOptions] = startOptions()
disp('App is executed, select your configuration there')
%% Create UI and populate
ui = CPMStartOptionsUI();

% CPM Lab
% FirstManualVehicleID
firstManualVehicleID = list_first_manual_vehicle();
ui.FirstManualVehicleMVIDListBox.Items = firstManualVehicleID(:,1);

% ControlModeFirstManualVehicle
controlMode = list_control_mode();
ui.ControlModeFirstMVListBox.Items = controlMode(:,2);

% SecondManualVehicleID
secondManualVehicleID = list_second_manual_vehicle();
ui.SecondMVIDListBox.Items = secondManualVehicleID(:,1);

% ControlModeSecondManualVehicle
secondControlMode = list_control_mode();
ui.ControlModeSecondMVListBox.Items = secondControlMode(:,2);

% Collision Avoidance
collisionAvoidance = list_collision_avoidance();
ui.CollisionAvoidanceListBox.Items = collisionAvoidance(:,2);


% Simulation
% scenario
scenario = list_scenario();
ui.ScenarioListBox.Items = scenario(:,2);

% controlStrategy
controlStrategy = list_control_strategy();
ui.ControlStrategyListBox.Items = controlStrategy(:,2);

% priorityAssignmentMethod
priorityAssignmentMethod = list_priority_assignment_methods();
ui.PriorityAssignmentMethodListBox.Items = priorityAssignmentMethod(:,2);

% vehicleAmount
vehicleAmount = list_vehicle_amount();
ui.AmountofVehiclesListBox.Items = vehicleAmount(:,1);

% visualization
visualization = list_visualization();
ui.TypeofVisualizationListBox_2.Items = visualization(:,2);

% isParl
isParl = list_is_parl();
ui.ParallelComputationListBox.Items = isParl(:,2);

% change visibility of vehicle ID selection depending on environment selection
ui.FirstManualVehicleMVIDListBox.ValueChangedFcn = @(~, ~) setControlModesVisibility(ui);
ui.SecondMVIDListBox.ValueChangedFcn = @(~, ~) setSecondControlModesVisibility(ui);
%ui.ControlStrategyListBox.ValueChangedFcn = @(~, ~) setVisualizationVisibility(ui);
ui.EnvironmentButtonGroup.SelectionChangedFcn = @(~, ~) setCpmLabElementsVisibility(ui);
ui.TrafficModeButtonGroup.SelectionChangedFcn = @(~, ~) setMixedTrafficElementsVisibility(ui);
ui.ControlModeFirstMVListBox.ValueChangedFcn = @(~, ~) setForceFeedbackVisibility(ui);

%ui.ScenarioListBox.ValueChangedFcn = @(~, ~) checkScenarioVehiclesMatch(ui, scenarios);
ui.ScenarioListBox.ValueChangedFcn = @(~, ~) setIsParlVisibility(ui);

% If parallel computation is not used, the number of computation levels cannot be constraint.
ui.ParallelComputationListBox.ValueChangedFcn = @(~, ~) callbackParlSelected(ui);

%% load previous choices, if possible
try %#ok<TRYNC>
    previousSelection = load([tempdir 'scenarioControllerSelection']);
    ui.EnvironmentButtonGroup.SelectedObject = ui.EnvironmentButtonGroup.Buttons(previousSelection.environmentSelection);
    ui.TrafficModeButtonGroup.SelectedObject =  ui.TrafficModeButtonGroup.Buttons(previousSelection.trafficModeSelection);
    ui.ForceFeedbackFirstMVButtonGroup.SelectedObject = ui.ForceFeedbackFirstMVButtonGroup.Buttons(previousSelection.forceFeedbackSelection);
    ui.FirstManualVehicleMVIDListBox.Value = previousSelection.firstManualVehicleIDSelection;
    ui.ControlModeFirstMVListBox.Value = previousSelection.controlModeSelection;
    ui.SecondMVIDListBox.Value = previousSelection.secondManualVehicleIDSelection;
    ui.ControlModeSecondMVListBox.Value = previousSelection.secondControlModeSelection;
    ui.CollisionAvoidanceListBox.Value = previousSelection.collisionAvoidanceSelection;

    ui.ScenarioListBox.Value = previousSelection.scenarioSelection;
    ui.ControlStrategyListBox.Value = previousSelection.controlStrategySelection;
    ui.PriorityAssignmentMethodListBox.Value = previousSelection.priorityAssignmentMethodSelection;
    ui.AmountofVehiclesListBox.Value = previousSelection.vehicleAmountSelection;
    ui.TypeofVisualizationListBox_2.Value = previousSelection.visualizationSelection;
    ui.ParallelComputationListBox.Value = previousSelection.isParlSelection;

    % sample time [s]
    ui.SampleTimesSpinner.Value = previousSelection.dtSelection;
    
    % predicion horizon
    ui.PredictionHorizonSpinner.Value = previousSelection.HpSelection;
    
    % MPA trim ID
    ui.MPAtrimIDSpinner.Value = previousSelection.trim_setSelection;

    % simulation duration [s]
    ui.SimulationDurationsSpinner.Value = previousSelection.T_endSelection;
    
    % maximum allowed number of computation levels
    ui.MaxComputationLevelsSpinner.Value = previousSelection.max_num_CLsSelection;
    
    % Stategy to let vehicle with the right-of-way consider vehicle without the right-of-way
    ui.HowShouldVehiclewiththeRightofWayConsiderVehicleWithoutListBox.Value = previousSelection.strategy_consider_veh_without_ROWSelection;
    
    % Strategy to let vehicle without the right-of-way enter the intersecting area of its lanelet with lanelet of its coupled vehicle
    ui.VehiclewithoutrightofwayEntersLaneletIntersectingAreaListBox.Value = previousSelection.strategy_enter_intersecting_areaSelection ;
end

%% Trigger UI change handles
setCpmLabElementsVisibility(ui);

%% Run App
% wait until UI is closed or Start pressed
%   emulating `waitfor(ui)`
while true
    if ~isvalid(ui) % if app close
        error('app closed, please press start button instead')
    elseif ui.StartButton.Value % start button pressed
        break
    end
    pause(0.1)
end
disp('Configuration Setup App finished, extracting data')

%% Extract Choices & Save for next time
firstManualVehicleIDSelection = ui.FirstManualVehicleMVIDListBox.Value;
secondManualVehicleIDSelection = ui.SecondMVIDListBox.Value;
controlModeSelection = ui.ControlModeFirstMVListBox.Value;
secondControlModeSelection = ui.ControlModeSecondMVListBox.Value;
collisionAvoidanceSelection = ui.CollisionAvoidanceListBox.Value;
environmentSelection = get_environment_selection(ui);
trafficModeSelection = get_traffic_mode_selection(ui);
forceFeedbackSelection = get_force_feedback_selection(ui);

scenarioSelection = ui.ScenarioListBox.Value;
controlStrategySelection = ui.ControlStrategyListBox.Value;
priorityAssignmentMethodSelection = ui.PriorityAssignmentMethodListBox.Value;
vehicleAmountSelection = ui.AmountofVehiclesListBox.Value;
visualizationSelection = ui.TypeofVisualizationListBox_2.Value;
isParlSelection = ui.ParallelComputationListBox.Value;

% sample time [s]
dtSelection = ui.SampleTimesSpinner.Value;
% predicion horizon
HpSelection = ui.PredictionHorizonSpinner.Value;
% MPA trim ID
trim_setSelection = ui.MPAtrimIDSpinner.Value;
% simulation duration [s]
T_endSelection = ui.SimulationDurationsSpinner.Value;
% maximum allowed number of computation levels
max_num_CLsSelection = ui.MaxComputationLevelsSpinner.Value;
% Stategy to let vehicle with the right-of-way consider vehicle without the right-of-way
strategy_consider_veh_without_ROWSelection = ui.HowShouldVehiclewiththeRightofWayConsiderVehicleWithoutListBox.Value;
% Strategy to let vehicle without the right-of-way enter the intersecting area of its lanelet with lanelet of its coupled vehicle
strategy_enter_intersecting_areaSelection = ui.VehiclewithoutrightofwayEntersLaneletIntersectingAreaListBox.Value;

save([tempdir 'scenarioControllerSelection'], 'firstManualVehicleIDSelection', 'controlModeSelection', 'secondManualVehicleIDSelection', 'secondControlModeSelection', 'collisionAvoidanceSelection',...
    'environmentSelection', 'trafficModeSelection', 'forceFeedbackSelection', 'scenarioSelection', 'controlStrategySelection', 'priorityAssignmentMethodSelection', 'vehicleAmountSelection', 'visualizationSelection',...
    'isParlSelection', 'dtSelection','HpSelection','trim_setSelection','T_endSelection','max_num_CLsSelection','strategy_consider_veh_without_ROWSelection','strategy_enter_intersecting_areaSelection');



% save([tempdir 'scenarioControllerSelection'], 'firstManualVehicleIDSelection', 'controlModeSelection', 'secondManualVehicleIDSelection', 'secondControlModeSelection',...
%     'environmentSelection', 'scenarioSelection', 'controlStrategySelection', 'priorityAssignmentMethodSelection', 'vehicleAmountSelection', 'visualizationSelection', 'isParlSelection',...
%     'dtSelection','HpSelection','T_endSelection','max_num_CLsSelection','strategy_consider_veh_without_ROWSelection','strategy_enter_intersecting_areaSelection');

%% Convert to legacy/outputs
labOptions.manualVehicle_id = firstManualVehicleID{...
    strcmp({firstManualVehicleID{:, 1}}, firstManualVehicleIDSelection),...
    1};

labOptions.firstManualVehicleMode = controlMode{...
    strcmp({controlMode{:, 2}}, controlModeSelection),...
    1}; 

labOptions.manualVehicle_id2 = secondManualVehicleID{...
    strcmp({secondManualVehicleID{:, 1}}, secondManualVehicleIDSelection),...
    1};

labOptions.secondManualVehicleMode = secondControlMode{...
    strcmp({secondControlMode{:, 2}}, secondControlModeSelection),...
    1};

labOptions.collisionAvoidanceMode = str2num(collisionAvoidance{...
    strcmp({collisionAvoidance{:, 2}}, collisionAvoidanceSelection),...
    1});

labOptions.is_sim_lab = ~get_environment_selection(ui, true);

labOptions.is_mixed_traffic = get_traffic_mode_selection(ui, true);

labOptions.force_feedback_enabled = get_force_feedback_selection(ui, true);

controlStrategyHelper = controlStrategy{...
    strcmp({controlStrategy{:, 2}}, controlStrategySelection),...
    2};
labOptions.isPB = (strcmp(controlStrategyHelper, 'pb non-coop'));

labOptions.angles = vehicleAmount{...
    strcmp({vehicleAmount{:,1}}, vehicleAmountSelection),...
    2};
    
labOptions.amount = str2num(vehicleAmountSelection);

labOptions.visu = visualization{...
    strcmp({visualization{:, 2}}, visualizationSelection),...
    3};

isParlHelper = isParl{...
strcmp({isParl{:, 2}}, isParlSelection),...
    2};

labOptions.isParl = strcmp(isParlHelper, 'yes');

% visualization + node exploration only allowed for centralized controller
if labOptions.isPB == 2 && strcmp(visualizationSelection, 'visualization + node exploration')
    labOptions.visu = visualization{2,3};
end

labOptions.scenario = scenario{...
    strcmp({scenario{:, 2}}, scenarioSelection),...
    2};
labOptions.priority = priorityAssignmentMethod{...
    strcmp({priorityAssignmentMethod{:, 2}}, priorityAssignmentMethodSelection),...
    2};

% sample time [s]
labOptions.dt = dtSelection;

% predicion horizon
labOptions.Hp = HpSelection;

% MPA trim ID
labOptions.trim_set = trim_setSelection;

% simulation duration [s]
labOptions.T_end = T_endSelection;

% maximum allowed number of computation levels
labOptions.max_num_CLs = max_num_CLsSelection;

% Stategy to let vehicle with the right-of-way consider vehicle without the right-of-way
labOptions.strategy_consider_veh_without_ROW = strategy_consider_veh_without_ROWSelection;

% Strategy to let vehicle without the right-of-way enter the intersecting area of its lanelet with lanelet of its coupled vehicle
labOptions.strategy_enter_intersecting_area = strategy_enter_intersecting_areaSelection;

% close app
ui.delete;
end


function out = get_environment_selection(ui, output_as_bool)
    % selection of environment
    out = ui.EnvironmentButtonGroup.SelectedObject == ui.EnvironmentButtonGroup.Buttons;
    
    % is CPM lab selected
    if nargin > 1 && output_as_bool
        out = isequal([1 0], out);
    end
end

function out = get_traffic_mode_selection(ui, output_as_bool)
    % selection of traffic mode
    out = ui.TrafficModeButtonGroup.SelectedObject == ui.TrafficModeButtonGroup.Buttons;
    
    % is mixed traffic selected
    if nargin > 1 && output_as_bool
        out = isequal([1 0], out);
    end
end

function out = get_force_feedback_selection(ui, output_as_bool)
    % selection of environment
    out = ui.ForceFeedbackFirstMVButtonGroup.SelectedObject == ui.ForceFeedbackFirstMVButtonGroup.Buttons;
    
    % is force feedback selected
    if nargin > 1 && output_as_bool
        out = isequal([1 0], out);
    end
end

function out = get_first_manual_vehicle_selection(ui)
    % true if no manual vehicle selected
    out = strcmp(ui.FirstManualVehicleMVIDListBox.Value, 'No MV');
end

function out = get_first_control_mode_selection(ui)
    % true if Guided-Mode selected
    out = strcmp(ui.ControlModeFirstMVListBox.Value, 'Guided-Mode');
end

function out = get_second_manual_vehicle_selection(ui)
    % true if no second manual vehicle selected
    out = strcmp(ui.SecondMVIDListBox.Value, 'No second MV');
end

function out = get_circle_selection(ui)
    %% true if circle scenario is selected
    out = strcmp(ui.ScenarioListBox.Value, 'Circle_scenario');
end

function out = get_pb_non_coop_selection(ui)
    % true if pb_non-coop lab selected
    out = strcmp(ui.ControlStrategyListBox.Value, 'pb non-coop');
end

function setCpmLabElementsVisibility(ui)
    % if lab mode is selected
    if get_environment_selection(ui, true)
        ui.TrafficModeButtonGroup.Visible = 'On';

        ui.ScenarioListBox.Enable = 'Off';
        ui.ControlStrategyListBox.Enable = 'Off';
        ui.PriorityAssignmentMethodListBox.Enable = 'Off';
        ui.AmountofVehiclesListBox.Enable = 'Off';
        ui.TypeofVisualizationListBox_2.Enable = 'Off';
        ui.ParallelComputationListBox.Enable = 'Off';      
    else
        ui.TrafficModeButtonGroup.Visible = 'Off';
        ui.FirstManualVehicleMVIDListBox.Enable = 'Off';
        ui.ControlModeFirstMVListBox.Enable = 'Off';
        ui.SecondMVIDListBox.Enable = 'Off';
        ui.ControlModeSecondMVListBox.Enable = 'Off';
        ui.CollisionAvoidanceListBox.Enable = 'Off';
        ui.ForceFeedbackFirstMVButtonGroup.Visible = 'Off';

        ui.ScenarioListBox.Enable = 'On';
        ui.ControlStrategyListBox.Enable = 'On';
        ui.PriorityAssignmentMethodListBox.Enable = 'On';
        ui.AmountofVehiclesListBox.Enable = 'On';
        ui.TypeofVisualizationListBox_2.Enable = 'On';
        ui.ParallelComputationListBox.Enable = 'On';
    end
end

function setMixedTrafficElementsVisibility(ui)
    % if mixed traffic is selected
    if get_traffic_mode_selection(ui, true)
        ui.FirstManualVehicleMVIDListBox.Enable = 'On';
        ui.ControlModeFirstMVListBox.Enable = 'On';
        ui.SecondMVIDListBox.Enable = 'On';
        ui.ControlModeSecondMVListBox.Enable = 'On';
        ui.CollisionAvoidanceListBox.Enable = 'On';
        ui.ForceFeedbackFirstMVButtonGroup.Visible = 'On';

        ui.ScenarioListBox.Enable = 'Off';
        ui.ControlStrategyListBox.Enable = 'Off';
        ui.PriorityAssignmentMethodListBox.Enable = 'Off';
        ui.AmountofVehiclesListBox.Enable = 'Off';
        ui.TypeofVisualizationListBox_2.Enable = 'Off';
        ui.ParallelComputationListBox.Enable = 'Off'; 
    else
        ui.FirstManualVehicleMVIDListBox.Enable = 'Off';
        ui.ControlModeFirstMVListBox.Enable = 'Off';
        ui.SecondMVIDListBox.Enable = 'Off';
        ui.ControlModeSecondMVListBox.Enable = 'Off';
        ui.CollisionAvoidanceListBox.Enable = 'Off';
        ui.ForceFeedbackFirstMVButtonGroup.Visible = 'Off';

        ui.ScenarioListBox.Enable = 'On';
        ui.ControlStrategyListBox.Enable = 'On';
        ui.PriorityAssignmentMethodListBox.Enable = 'On';
        ui.AmountofVehiclesListBox.Enable = 'Off';
        ui.TypeofVisualizationListBox_2.Enable = 'Off';
        ui.ParallelComputationListBox.Enable = 'On';
    end
end

function setControlModesVisibility(ui)
    % if no manual vehicle is selected
    if get_first_manual_vehicle_selection(ui)
        ui.ControlModeFirstMVListBox.Enable = 'Off';
        ui.SecondMVIDListBox.Enable = 'Off';
        ui.ControlModeSecondMVListBox.Enable = 'Off';
        ui.CollisionAvoidanceListBox.Enable = 'Off';
        ui.ForceFeedbackFirstMVButtonGroup.Visible = 'Off';
    else
        ui.ControlModeFirstMVListBox.Enable = 'On';
        ui.SecondMVIDListBox.Enable = 'On'; 
        ui.ControlModeSecondMVListBox.Enable = 'On';
        ui.CollisionAvoidanceListBox.Enable = 'On';
    end
end

function setSecondControlModesVisibility(ui)
    if get_second_manual_vehicle_selection(ui)
        ui.ControlModeSecondMVListBox.Enable = 'Off';
    else
        ui.ControlModeSecondMVListBox.Enable = 'On';
    end
end

function setForceFeedbackVisibility(ui)
    if get_first_control_mode_selection(ui)
        ui.ForceFeedbackFirstMVButtonGroup.Visible = 'On';
    else
        ui.ForceFeedbackFirstMVButtonGroup.Visible = 'Off';
    end
end

function setIsParlVisibility(ui)
    if get_circle_selection(ui)
        ui.ParallelComputationListBox.Enable = 'Off';
        ui.Label_4.Text = sprintf("For circle scenario, only\n topo priority, constant\n priority and random\n priority are supported");
        ui.Label_4.Visible = 'On';
    else
        ui.ParallelComputationListBox.Enable = 'On';
        ui.Label_4.Visible = 'Off';
    end
end

% callback function if parallel computation is selected/unselected
function callbackParlSelected(ui)
    if strcmp(ui.ParallelComputationListBox.Value,'yes')
        ui.MaxComputationLevelsSpinner.Enable = 'on';
        ui.HowShouldVehiclewiththeRightofWayConsiderVehicleWithoutListBox.Enable = 'on';
        ui.VehiclewithoutrightofwayEntersLaneletIntersectingAreaListBox.Enable = 'on';
%         ui.PriorityAssignmentMethodListBox.Value = ui.PriorityAssignmentMethodListBox.Items{2};

        ui.Label_4.Visible = 'Off';
    else
        ui.MaxComputationLevelsSpinner.Enable = 'off';
        ui.HowShouldVehiclewiththeRightofWayConsiderVehicleWithoutListBox.Enable = 'off';
        ui.VehiclewithoutrightofwayEntersLaneletIntersectingAreaListBox.Enable = 'off';

        ui.Label_4.Text = sprintf("If parallel computation is not used, the maximum allowed number of computation levels is irrelevant.");
        ui.Label_4.Visible = 'On';
    end
end

%{
function setVisualizationVisibility(ui)
    % no node exploration if pb non-coop is selected
    if get_pb_non_coop_selection(ui)
        set(ui.TypeofVisualizationListBox_2, 'visualization + node exploration', 'Off');
    else
        set(ui.TypeofVisualizationListBox_2, 'visualization + node exploration', 'On');
    end
end
%}

function [ list ] = list_first_manual_vehicle
    list = {...
    'No MV', pi+pi; ...
    '1', pi+pi*(1:2); ...
    '2', pi+2*pi/3*(1:3); ...
    '3', pi+2*pi/4*(1:4); ...
    '4', pi+2*pi/5*(1:5); ...
    '5', pi+2*pi/6*(1:6); ...
    '6', pi+2*pi/7*(1:7); ...
    '7', pi+2*pi/8*(1:8); ...
    '8', pi+2*pi/9*(1:10); ...
    '9', pi+2*pi/10*(1:10); ...
    '10', pi+2*pi/11*(1:11); ...
    '11', pi+2*pi/12*(1:12); ...
    '12', pi+2*pi/13*(1:13); ...
    '13', pi+2*pi/14*(1:14); ...
    '14', pi+2*pi/15*(1:15); ...
    '15', pi+2*pi/15*(1:16); ...
    '16', pi+2*pi/15*(1:17); ...
    '17', pi+2*pi/15*(1:18); ...
    '18', pi+2*pi/15*(1:19); ...
    '19', pi+2*pi/15*(1:20); ...
    '20', pi+2*pi/15*(1:21); ...
    };
end

function [ list ] = list_second_manual_vehicle
    list = {...
    'No second MV', pi+pi; ...
    '1', pi+pi*(1:2); ...
    '2', pi+2*pi/3*(1:3); ...
    '3', pi+2*pi/4*(1:4); ...
    '4', pi+2*pi/5*(1:5); ...
    '5', pi+2*pi/6*(1:6); ...
    '6', pi+2*pi/7*(1:7); ...
    '7', pi+2*pi/8*(1:8); ...
    '8', pi+2*pi/9*(1:10); ...
    '9', pi+2*pi/10*(1:10); ...
    '10', pi+2*pi/11*(1:11); ...
    '11', pi+2*pi/12*(1:12); ...
    '12', pi+2*pi/13*(1:13); ...
    '13', pi+2*pi/14*(1:14); ...
    '14', pi+2*pi/15*(1:15); ...
    '15', pi+2*pi/15*(1:16); ...
    '16', pi+2*pi/15*(1:17); ...
    '17', pi+2*pi/15*(1:18); ...
    '18', pi+2*pi/15*(1:19); ...
    '19', pi+2*pi/15*(1:20); ...
    '20', pi+2*pi/15*(1:21); ...
    };
end

function [ list ] = list_collision_avoidance
    list = {...
    '1', 'Priority-based'; ...
    '2', 'Reachability Analysis Guided-Mode'; ...
    '3', 'Reachability Analysis Expert-Mode'; ...
    };
end

function [ list ] = list_control_mode
    list = {...
    '1', 'Guided-Mode'; ...
    '2', 'Expert-Mode'; ...
    };
end

function [ list ] = list_scenario
    list = {...
    '1','Circle_scenario';...
    '2','Commonroad'...
    };
end

function [ list ] = list_control_strategy
    list = {...
    '1', 'centralized'; ...
    '2', 'pb non-coop'; ...
    };
end

function [ list ] = list_priority_assignment_methods
    list = {...
    '1','topo_priority';...
    '2','right_of_way_priority'
    '2','constant_priority';...
    '3','random_priority';...
    '4','FCA_priority'
    };
end 

function [ list ] = list_visualization
    list = {...
    '1', 'no visualization',                    [false,false]; ...
    '2', 'vehicle visualization',               [true,false]; ...
    '3', 'visualization + node exploration',    [true,true]; ... % only for centralized controller
    };
end

function [ list ] = list_vehicle_amount
    list = {...
    '1', pi+pi; ...
    '2', pi+pi*(1:2); ...
    '3', pi+2*pi/3*(1:3); ...
    '4', pi+2*pi/4*(1:4); ...
    '5', pi+2*pi/5*(1:5); ...
    '6', pi+2*pi/6*(1:6); ...
    '7', pi+2*pi/7*(1:7); ...
    '8', pi+2*pi/8*(1:8); ...
    '9', pi+2*pi/9*(1:10); ...
    '10', pi+2*pi/10*(1:10); ...
    '11', pi+2*pi/11*(1:11); ...
    '12', pi+2*pi/12*(1:12); ...
    '13', pi+2*pi/13*(1:13); ...
    '14', pi+2*pi/14*(1:14); ...
    '15', pi+2*pi/15*(1:15); ...
    '16', pi+2*pi/15*(1:16); ...
    '17', pi+2*pi/15*(1:17); ...
    '18', pi+2*pi/15*(1:18); ...
    '19', pi+2*pi/15*(1:19); ...
    '20', pi+2*pi/15*(1:20); ...
    };
end

function [ list ] = list_is_parl
    list = {...
    '1', 'yes'; ...
    '2', 'no'; ...
    };
end


