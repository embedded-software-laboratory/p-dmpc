function [new_open_nodes] = expand_node(scenario, iter, iNode, info)
% EXPAND_NODE   Expand node in search tree and return succeeding nodes.
    trim_tuple = scenario.mpa.trim_tuple;
    trim_length = length(scenario.mpa.trims)*ones(1, iter.amount);
    
    curX     = info.tree.x(:,iNode);
    curY     = info.tree.y(:,iNode);
    curYaw   = info.tree.yaw(:,iNode);
    curTrim  = info.tree.trim(:,iNode);
    curK     = info.tree.k(:,iNode);
    curG     = info.tree.g(:,iNode);

    k_exp = curK+1;
    cur_trim_id = tuple2index(curTrim(:),trim_length);
    % TODO: mpa Verkleinerung bug fix
    successor_trim_ids = find(scenario.mpa.transition_matrix(cur_trim_id, :, k_exp));

    nTrims = numel(successor_trim_ids);
    
    expX     = zeros(iter.amount,nTrims);
    expY     = zeros(iter.amount,nTrims);
    expYaw   = zeros(iter.amount,nTrims);
    expTrim  = zeros(iter.amount,nTrims);
    expK     = k_exp*ones(1,nTrims);
    expG     = curG*ones(1,nTrims);
    expH     = zeros(1,nTrims);
    

    time_steps_to_go = scenario.options.Hp - k_exp;
    for iTrim = 1:nTrims
        id = successor_trim_ids(iTrim);
        expTrim(:,iTrim) = trim_tuple(id,:);
        for iVeh = 1 : iter.amount
            itrim1 = curTrim(iVeh);
            itrim2 = expTrim(iVeh,iTrim);

            % if current vehicle is manual vehicle and its MPA is already initialized, choose the corresponding MPA
            maneuver = scenario.mpa.maneuvers{itrim1, itrim2};
            
            c = cos(curYaw(iVeh));
            s = sin(curYaw(iVeh));

            expX(iVeh,iTrim) = c*maneuver.dx - s*maneuver.dy + curX(iVeh);
            expY(iVeh,iTrim) = s*maneuver.dx + c*maneuver.dy + curY(iVeh);
            expYaw(iVeh,iTrim) = curYaw(iVeh) + maneuver.dyaw;

            % Cost to come
            % iter.reference is of size (scenario.options.amount,scenario.options.Hp,2)
            % Distance to reference trajectory points squared to conform with
            % J = (x-x_ref)' Q (x-x_ref)
            expG(iTrim) = expG(iTrim) + norm([expX(iVeh,iTrim) - iter.referenceTrajectoryPoints(iVeh,k_exp,1);expY(iVeh,iTrim) - iter.referenceTrajectoryPoints(iVeh,k_exp,2)])^2;


            % Cost to go
            % same as cost to come
            % subtract squared distance traveled for every timestep and vehicle
            d_traveled_max = 0;
            for i_t = 1:time_steps_to_go
                d_traveled_max = d_traveled_max...
                    + scenario.options.dt*iter.v_ref(iVeh,k_exp+i_t);
                expH(iTrim) = expH(iTrim) + (norm( [expX(iVeh,iTrim)-iter.referenceTrajectoryPoints(iVeh,k_exp+i_t,1); expY(iVeh,iTrim)-iter.referenceTrajectoryPoints(iVeh,k_exp+i_t,2)] ) - d_traveled_max)^2;
            end
            
%             % vectorize the for-loop that calculates the cost-to-go (if
%             needed)
%             distancesTraveledMax = cumsum(scenario.options.dt*iter.vRef(iVeh,k_exp+1:end));
%             distancesXy = [expX(iVeh,iTrim);expY(iVeh,iTrim)] - [iter.referenceTrajectoryPoints(iVeh,k_exp+1:end,1);iter.referenceTrajectoryPoints(iVeh,k_exp+1:end,2)];
%             straightLineDistances = sqrt(sum(distancesXy.^2,1));
%             expH(iTrim) = sum((distancesTraveledMax-straightLineDistances).^2);
        end
    end
%     expG = ones(1,nTrims);
%     expG(1) = -2;
%     expG(2) = -2;
%     expH = 0*ones(1,nTrims);
%     expH(1) = -2;
    new_open_nodes = add_nodes(info.tree,iNode,expX,expY,expYaw,expTrim,expK,expG,expH);
end
