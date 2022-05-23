function iter = filter_iter(iter, vehicle_filter)
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
    iter.vRef = iter.vRef(vehicle_filter,:);
    iter.referenceTrajectoryPoints = iter.referenceTrajectoryPoints(vehicle_filter,:,:);
    iter.referenceTrajectoryIndex = iter.referenceTrajectoryIndex(vehicle_filter,:,:);
    iter.predicted_lanelets = iter.predicted_lanelets(vehicle_filter);
    iter.predicted_lanelet_boundary = iter.predicted_lanelet_boundary(vehicle_filter,:);
    iter.reachable_sets = iter.reachable_sets(vehicle_filter,:);

    
end