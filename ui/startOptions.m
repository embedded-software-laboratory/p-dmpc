function [labOptions] = startOptions()
disp('App is executed, select your configuration there')
%% Create UI and populate
ui = CPMStartOptionsUI();

% Simulation
% scenario
scenario = list_scenario();
ui.ScenarioListBox.Items = scenario(:,2);

% controlStrategy
controlStrategy = list_control_strategy();
ui.ControlStrategyListBox.Items = controlStrategy(:,2);
ui.ControlStrategyListBox.ValueChangedFcn = @(~, ~) callbackPBSelected(ui);

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
ui.ParallelComputationDistributedExecutionListBox.Items = isParl(:,2);

% change visibility of vehicle ID selection depending on environment selection
%ui.ControlStrategyListBox.ValueChangedFcn = @(~, ~) setVisualizationVisibility(ui);
ui.EnvironmentButtonGroup.SelectionChangedFcn = @(~, ~) setCpmLabElementsVisibility(ui);
ui.AddHDVsCheckBox.ValueChangedFcn = @(~, ~) setManualControlElementsVisibility(ui);

%% load previous choices, if possible
try %#ok<TRYNC>
    previousSelection = load([tempdir 'scenarioControllerSelection']);
    ui.AddHDVsCheckBox.Value = previousSelection.is_manual_control;
    ui.AmountHDVsListBox.Value = previousSelection.hdv_amount_selection;
    ui.HDVIDsEditField.Value = previousSelection.hdv_ids;
    ui.EnvironmentButtonGroup.SelectedObject = ui.EnvironmentButtonGroup.Buttons(previousSelection.environmentSelection);
    ui.ParallelComputationDistributedExecutionListBox.Value = previousSelection.isParlSelection;
    ui.CustomVehicleIdsEditField.Value = previousSelection.veh_ids;

    ui.ScenarioListBox.Value = previousSelection.scenarioSelection;
    ui.ControlStrategyListBox.Value = previousSelection.controlStrategySelection;
    ui.PriorityAssignmentMethodListBox.Value = previousSelection.priorityAssignmentMethodSelection;
    ui.AmountofVehiclesListBox.Value = previousSelection.vehicleAmountSelection;
    ui.TypeofVisualizationListBox_2.Value = previousSelection.visualizationSelection;

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
    
    % Strategy to let vehicle without the right-of-way enter the crossing area of its lanelet with lanelet of its coupled vehicle
    ui.VehiclewithoutrightofwayEntersLaneletCrossingAreaListBox.Value = previousSelection.strategy_enter_crossing_areaSelection ;

    % Whether save result
    ui.SaveresultCheckBox.Value = previousSelection.isSaveResult;

    % Custom file name
    ui.CustomfilenameEditField.Value = previousSelection.customResultName;

    % Whether vehicles are allowed to inherit the right-of-way from their front vehicles
    ui.AllowInheritingtheRightofWayCheckBox.Value = previousSelection.isAllowInheritROW;
end

%% Trigger UI change handles
setCpmLabElementsVisibility(ui);
setManualControlElementsVisibility(ui);

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

scenarioSelection = ui.ScenarioListBox.Value;
controlStrategySelection = ui.ControlStrategyListBox.Value;
priorityAssignmentMethodSelection = ui.PriorityAssignmentMethodListBox.Value;
vehicleAmountSelection = ui.AmountofVehiclesListBox.Value;
visualizationSelection = ui.TypeofVisualizationListBox_2.Value;
isParlSelection = ui.ParallelComputationDistributedExecutionListBox.Value;
veh_ids = ui.CustomVehicleIdsEditField.Value;
hdv_ids = ui.HDVIDsEditField.Value;
hdv_amount_selection = ui.AmountHDVsListBox.Value;
is_manual_control = ui.AddHDVsCheckBox.Value;


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
% Strategy to let vehicle without the right-of-way enter the crossing area of its lanelet with lanelet of its coupled vehicle
strategy_enter_crossing_areaSelection = ui.VehiclewithoutrightofwayEntersLaneletCrossingAreaListBox.Value;
% Whether save result
isSaveResult = ui.SaveresultCheckBox.Value;
% Custom file name
customResultName = ui.CustomfilenameEditField.Value;
% Whether vehicles are allowed to inherit the right-of-way from their front vehicles
isAllowInheritROW = ui.AllowInheritingtheRightofWayCheckBox.Value;
save([tempdir 'scenarioControllerSelection'], 'is_manual_control', 'hdv_amount_selection', 'hdv_ids', ...
    'environmentSelection', 'scenarioSelection', 'controlStrategySelection', 'priorityAssignmentMethodSelection', 'vehicleAmountSelection', 'visualizationSelection',...
    'isParlSelection', 'dtSelection','HpSelection','trim_setSelection','T_endSelection','max_num_CLsSelection', 'veh_ids', 'strategy_consider_veh_without_ROWSelection','strategy_enter_crossing_areaSelection', ...
    'isSaveResult','customResultName','isAllowInheritROW');

%% Convert to legacy/outputs
% initialize
labOptions = Config();


manual_control_config = ManualControlConfig;

manual_control_config.amount = str2double(hdv_amount_selection)*is_manual_control;

hdv_ids = ui.HDVIDsEditField.Value;
if hdv_ids ~= "" && is_manual_control
    hdv_ids(~isstrprop(hdv_ids,'digit')) = ' '; %replace non-numeric characters with empty space
    manual_control_config.hdv_ids = str2double(strsplit(strtrim(hdv_ids)));
else
    manual_control_config.hdv_ids = [];
end

assert(length(manual_control_config.hdv_ids)*is_manual_control == manual_control_config.amount*is_manual_control, ['Type in exactly ', num2str(manual_control_config.amount), ' HDV ID(s)']);


%labOptions.is_eval = false;

%labOptions.visualize_reachable_set = false;

labOptions.manual_control_config = manual_control_config;

labOptions.is_sim_lab = ~get_environment_selection(ui, true);

controlStrategyHelper = controlStrategy{...
    strcmp({controlStrategy{:, 2}}, controlStrategySelection),...
    2};

labOptions.isPB = (strcmp(controlStrategyHelper, 'pb non-coop'));

labOptions.amount = str2num(vehicleAmountSelection);

veh_ids = ui.CustomVehicleIdsEditField.Value;
if veh_ids ~= ""
    veh_ids(~isstrprop(veh_ids,'digit')) = ' '; %replace non-numeric characters with empty space
    labOptions.veh_ids = str2double(strsplit(strtrim(veh_ids)));
else
    labOptions.veh_ids = [];
end

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

labOptions.scenario_name = scenario{...
    strcmp({scenario{:, 2}}, scenarioSelection),...
    1};

labOptions.priority = priorityAssignmentMethod{...
    strcmp({priorityAssignmentMethod{:, 2}}, priorityAssignmentMethodSelection),...
    1};

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

% Strategy to let vehicle without the right-of-way enter the crossing area of its lanelet with lanelet of its coupled vehicle
labOptions.strategy_enter_lanelet_crossing_area = strategy_enter_crossing_areaSelection;

% Whether save result
labOptions.isSaveResult = ui.SaveresultCheckBox.Value;

if labOptions.isSaveResult
    disp('As required, simulation/Experiment Results will not be saved.')
end

% Custom file name to save result
labOptions.customResultName = ui.CustomfilenameEditField.Value;

% Whether vehicles are allowed to inherit the right-of-way from their front vehicles
labOptions.isAllowInheritROW = ui.AllowInheritingtheRightofWayCheckBox.Value;

% Write Config to disk
encodedJSON = jsonencode(labOptions);
fid = fopen('Config.json','w');
fprintf(fid, encodedJSON);
fclose('all');
% save('config.mat','labOptions');

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

function out = get_add_hdv_selection(ui)
    % selection of traffic mode
    out = ui.AddHDVsCheckBox.Value;
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
    is_cpm_lab_selection = get_environment_selection(ui, true);
    ui.AddHDVsCheckBox.Enable = is_cpm_lab_selection;
    ui.AddHDVsCheckBox.Value = ui.AddHDVsCheckBox.Value && is_cpm_lab_selection;
    ui.AmountHDVsListBox.Enable = ui.AmountHDVsListBox.Enable && is_cpm_lab_selection;
    ui.HDVIDsEditField.Enable = ui.HDVIDsEditField.Enable && is_cpm_lab_selection;
end

function setManualControlElementsVisibility(ui)
    % if hdvs should be added
    is_add_hdvs = get_add_hdv_selection(ui);
    ui.AmountHDVsListBox.Enable = is_add_hdvs;
    ui.HDVIDsEditField.Enable = is_add_hdvs;
end
 
% callback function if parallel computation is selected/unselected 
function callbackPBSelected(ui) 
    if strcmp(ui.ControlStrategyListBox.Value,'pb non-coop') 
        ui.MaxComputationLevelsSpinner.Enable = 'on'; 
        ui.HowShouldVehiclewiththeRightofWayConsiderVehicleWithoutListBox.Enable = 'on'; 
        ui.VehiclewithoutrightofwayEntersLaneletCrossingAreaListBox.Enable = 'on';
        ui.ParallelComputationDistributedExecutionListBox.Enable = 'on'; 

        ui.Label_4.Visible = 'Off'; 
    else 
        ui.MaxComputationLevelsSpinner.Enable = 'off'; 
        ui.HowShouldVehiclewiththeRightofWayConsiderVehicleWithoutListBox.Enable = 'off'; 
        ui.VehiclewithoutrightofwayEntersLaneletCrossingAreaListBox.Enable = 'off';
        ui.ParallelComputationDistributedExecutionListBox.Enable = 'off'; 
 
        ui.Label_4.Text = sprintf("If not priority-based, the maximum allowed number of computation levels is irrelevant."); 
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
 
function [ list ] = list_is_parl 
    list = {... 
    '1', 'yes'; ... 
    '2', 'no'; ... 
    }; 
end 

function [ list ] = list_scenario
    list = {...
    'Circle_scenario','Circle Scenario';...
    'Commonroad','Commonroad'...
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
    'coloring_priority','Coloring Priority';...
    'right_of_way_priority','Right of way Priority'
    'constant_priority','Constant Priority';...
    'random_priority','Random Priority';...
    'FCA_priority','FCA Priority';...
    'STAC_priority','STAC Priority'
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
    '16', pi+2*pi/16*(1:16); ...
    '17', pi+2*pi/17*(1:17); ...
    '18', pi+2*pi/18*(1:18); ...
    '19', pi+2*pi/19*(1:19); ...
    '20', pi+2*pi/20*(1:20); ...
    '21', pi+2*pi/21*(1:21); ...
    '22', pi+2*pi/22*(1:22); ...
    '23', pi+2*pi/23*(1:23); ...
    '24', pi+2*pi/24*(1:24); ...
    '25', pi+2*pi/25*(1:25); ...
    '26', pi+2*pi/26*(1:26); ...
    '27', pi+2*pi/27*(1:27); ...
    '28', pi+2*pi/28*(1:28); ...
    '29', pi+2*pi/29*(1:29); ...
    '30', pi+2*pi/30*(1:30); ...
    '31', pi+2*pi/31*(1:31); ...
    '32', pi+2*pi/32*(1:32); ...
    '33', pi+2*pi/33*(1:33); ...
    '34', pi+2*pi/34*(1:34); ...
    '35', pi+2*pi/35*(1:35); ...
    '36', pi+2*pi/36*(1:36); ...
    '37', pi+2*pi/37*(1:37); ...
    '38', pi+2*pi/38*(1:38); ...
    '39', pi+2*pi/39*(1:39); ...
    '40', pi+2*pi/40*(1:40); ...
    };
end
