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

    % Information from previous time step extract
    if ~isempty(fieldnames(iter.info_prev))
        iter.info_prev.vehicle_fullres_path = iter.info_prev.vehicle_fullres_path(vehicle_filter,:);
        iter.info_prev.trim_indices = iter.info_prev.trim_indices(vehicle_filter);
        iter.info_prev.subcontroller_runtime = iter.info_prev.subcontroller_runtime(vehicle_filter);
        iter.info_prev.shapes = iter.info_prev.shapes(vehicle_filter,:);
        iter.info_prev.trims_Hp = iter.info_prev.trims_Hp(vehicle_filter,:);
        iter.info_prev.tree_path = iter.info_prev.tree_path(vehicle_filter,:);
        iter.info_prev.y_predicted = iter.info_prev.y_predicted(vehicle_filter,:);
        iter.info_prev.trees = iter.info_prev.trees(vehicle_filter);
        iter.info_prev.n_exhausted = iter.info_prev.n_exhausted(vehicle_filter);
%         if sum(vehicle_filter)==0
%             iter.info_prev.trees = struct;
%         elseif sum(vehicle_filter)==1 % if only one vehicle is selected, reduce the cell array of trees to a single tree
%             iter.info_prev.tree = iter.info_prev.trees{vehicle_filter}; % create new field named 'tree'
%             iter.info_prev = rmfield(iter.info_prev,'trees'); % delete the field named 'trees'
%         else
%             iter.info_prev.trees = iter.info_prev.trees(vehicle_filter);
%         end
    end

    
end