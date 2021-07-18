function scenario = filter_scenario(scenario, vehicle_filter)
% FILTER_SCENARIO   Reduce scenario structure to selected vehicles.

% Copyright 2016 Bassam Alrifaee
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License version 3 as 
% published by the Free Software Foundation.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <http://www.gnu.org/licenses/>.

    scenario.vehicles = scenario.vehicles(vehicle_filter);
    scenario.vehicle_to_lanelet = scenario.vehicle_to_lanelet(vehicle_filter,:);
    scenario.nVeh = sum(vehicle_filter);
end