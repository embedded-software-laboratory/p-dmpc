classdef OptionsPlotOnline
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        isShowVehID = true;                 % is show vehicle IDs
        isShowPriority = true;              % is show priority colorbar
        isShowCoupling = 1;              % is show coupling edges
        isShowWeight = true;                % is show coupling weights
        isShowHotkeyDescription = false;    % is show description of hotkeys
        isShowReachableSets = false;
        vehsReachableSets = [];             % reachable sets of those vehicles will be shown
        isShowLaneletCrossingAreas = false; % is show lanelet crossing areas
        vehsLaneletCorssingAreas = [];      % lanelet corssing area of those vehicles will be shown
        isVideoMode = false;                % if in video mode, the lanelets are needed to be plotted for each time step 
    end

    methods
        function obj = OptionsPlotOnline()
        end
    end
end