function [dynamic_obstacle_reachableSets_new] = vehicles_as_obstacles_other_grps(dynamic_obstacle_reachableSets_old, iter_filtered_other_grps, reachable_sets_other_grps)
% VEHICLES_AS_OBSTACLES_OTHER_GRPS Add reachable sets of the coupled vehicles with
%                       higher priorities in other groups as obsticle
% Params:
%     scenario_v:
%         information of the ego vehicle
%     v2o_filter_other_grps:
%         ligical array, indicates which vehicles' reachable sets should be
%         considered by the ego vehicle 
%     reachable_sets_other_grps:
%         The reachable sets in the prediction horizon calculated offline using motion primitives
% Returns:
%     dynamic_obstacle_reachableSets: cell [numberOfCoupledVehiclesInOtherGroups x Hp]
%         
    
    assert( size(reachable_sets_other_grps,1) == size(iter_filtered_other_grps.x0,1) )

    nDynObstOld = size(dynamic_obstacle_reachableSets_old, 1); % number of old obsticles
    nDynObstNew = size(reachable_sets_other_grps,1);  % number of new obsticles
    Hp = size(reachable_sets_other_grps,2);
    dynamic_obstacle_reachableSets_new = [dynamic_obstacle_reachableSets_old; cell(nDynObstNew,Hp)];
    
    reachable_sets_other_grps_trans = cell(size(reachable_sets_other_grps));
    for iVeh=1:nDynObstNew
        for t=1:Hp
            yaw0  = iter_filtered_other_grps.x0(iVeh,NodeInfo.yaw);
            x0  = iter_filtered_other_grps.x0(iVeh,NodeInfo.x);
            y0  = iter_filtered_other_grps.x0(iVeh,NodeInfo.y);
            % translate the offline calculated lacal reachable sets to global coordinate
            [reachable_sets_other_grps_trans{iVeh,t}(1,:), reachable_sets_other_grps_trans{iVeh,t}(2,:)] = ...
                translate_global(yaw0,x0,y0,reachable_sets_other_grps{iVeh,t}.Vertices(:,1)',reachable_sets_other_grps{iVeh,t}.Vertices(:,2)');
        end
        % add reachable sets as obstacles
        dynamic_obstacle_reachableSets_new(nDynObstOld+iVeh,:) = reachable_sets_other_grps_trans(iVeh,:);
    end


end
