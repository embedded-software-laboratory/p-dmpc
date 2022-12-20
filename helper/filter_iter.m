function iter = filter_iter(iter, vehicle_filter) % TODO: Add new fields
% FILTER_ITER   Reduce iter structure to selected vehicles.

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

    iter.x0 = iter.x0(vehicle_filter,:);
    iter.trim_indices = iter.trim_indices(vehicle_filter);
    iter.v_ref = iter.v_ref(vehicle_filter,:);
    iter.referenceTrajectoryPoints = iter.referenceTrajectoryPoints(vehicle_filter,:,:);
    iter.referenceTrajectoryIndex = iter.referenceTrajectoryIndex(vehicle_filter,:,:);
    iter.predicted_lanelets = iter.predicted_lanelets(vehicle_filter);
    iter.predicted_lanelet_boundary = iter.predicted_lanelet_boundary(vehicle_filter,:);
    iter.reachable_sets = iter.reachable_sets(vehicle_filter,:);
    iter.emergency_maneuvers = iter.emergency_maneuvers(vehicle_filter);
    iter.last_trajectory_index = iter.last_trajectory_index(vehicle_filter);
    iter.lane_change_lanes = iter.lane_change_indices(vehicle_filter,:,:);
    iter.lane_change_indices = iter.lane_change_indices(vehicle_filter,:,:);
    iter.lanes_before_update = iter.lanes_before_update(vehicle_filter,:,:);
    iter.auto_updated_path = iter.auto_updated_path(vehicle_filter);
    iter.vehicle_to_lanelet = iter.vehicle_to_lanelet(vehicle_filter);
    iter.vehicles = iter.vehicles(vehicle_filter);
    iter.amount = sum(vehicle_filter);
end