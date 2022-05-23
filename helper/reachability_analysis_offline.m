function [reachable_sets_local, reachable_sets_conv_local] = reachability_analysis_offline(mpa, Hp)
% REACHABILITY_ANALYSIS_OFFLINE Calculate local reachable sets starting
% from a certain trim offline, which lragely accelerates the online
% reachaility analysis.
% 
% INPUT:
%   mpa: motion primitive automaton
%   
%   Hp: prediction horizon
% 
% OUTPUT:
%   reachable_sets_local: cell [n_trims x Hp]. The union of local reachable
%   sets  
%     
%   reachable_sets_conv_local: cell [n_trims x Hp]. The convexified union
%   of local reachable sets 
    
    threshold_Hp = 5;
    if Hp > threshold_Hp
        warning(['Computing the reachable sets now...' newline ...
            'Since the prediction horizon is ' num2str(Hp) ' (more than ' num2str(threshold_Hp) '), it may take several minutes.' newline ...
            'Note this only needs to be done once since later they will be saved for offline use.'])
    end

    n_trims = numel(mpa.trims);
    reachable_sets_local = cell(n_trims,Hp);
    reachable_sets_conv_local = cell(n_trims,Hp);

    % transform maneuver area to polyshape which is required when using
    % MATLAB function `union`
    for i=1:n_trims
        child_trims = find(mpa.transition_matrix_single(i,:,1));
        for idx=1:length(child_trims)
            j = child_trims(idx);
            mpa.maneuvers{i,j}.areaPoly = polyshape(mpa.maneuvers{i,j}.area(1,:),mpa.maneuvers{i,j}.area(2,:),'Simplify',false);
        end
    end
    
%     % todo: use a different horizon for reachability analysis
%     if Hp>5
%         Hp = 5;
%         warning(['The pridiction horizon is too large for reachability analysis and therefore a prediction horizon of ', num2str(Hp),' will be used.'])
%     end

    trimsInfo = struct;
    for i=1:n_trims
        for t=1:Hp
            if t==1 % root trim
                trimsInfo(i,t).parentTrims = i;
            else % The child trims become the parent trims of the next time step
                trimsInfo(i,t).parentTrims = trimsInfo(i,t-1).childTrims;
            end
            
            % variable to store all child trims of the parent trims
            trimsInfo(i,t).childTrims = [];
            trimsInfo(i,t).childNum = [];
            % find child trims of the parent trims
            for i_Trim=1:length(trimsInfo(i,t).parentTrims)
                find_child = find(mpa.transition_matrix_single(trimsInfo(i,t).parentTrims(i_Trim),:,t));
                trimsInfo(i,t).childTrims = [trimsInfo(i,t).childTrims find_child];
                trimsInfo(i,t).childNum = [trimsInfo(i,t).childNum length(find_child)];
            end
            
            % store the union of the reachable sets of the parent trims in the prediction horizon
            trimsInfo(i,t).reachable_sets = polyshape;
            % store the union of the reachable sets of one parent tim
            reachable_sets_local = cell(1,length(trimsInfo(i,t).parentTrims));
            
            % loop through all parent trims
            for j=1:length(trimsInfo(i,t).parentTrims)
                reachable_sets_local{j} = polyshape;
                % loop through all child trims
                for k=1:trimsInfo(i,t).childNum(j)
                    trim_start = trimsInfo(i,t).parentTrims(j);
                    child_ordinal = sum(trimsInfo(i,t).childNum(1:j-1)) + k;
                    trim_end = trimsInfo(i,t).childTrims(child_ordinal);
                    if t==1
                        x0 = 0;
                        y0 = 0;
                        yaw0 = 0;
                    else
                        x0 = trimsInfo(i,t-1).maneuvers{j}.dx;
                        y0 = trimsInfo(i,t-1).maneuvers{j}.dy;
                        yaw0 = trimsInfo(i,t-1).maneuvers{j}.dyaw;
                    end
    
                    % tranlates the local coordinates to global coordinates
                    [trimsInfo(i,t).maneuvers{child_ordinal}.xs, trimsInfo(i,t).maneuvers{child_ordinal}.ys] = ...
                        translate_global(yaw0,x0,y0,mpa.maneuvers{trim_start,trim_end}.xs,mpa.maneuvers{trim_start,trim_end}.ys);
                    trimsInfo(i,t).maneuvers{child_ordinal}.yaws = yaw0 + mpa.maneuvers{trim_start,trim_end}.yaws;
                    trimsInfo(i,t).maneuvers{child_ordinal}.dx = trimsInfo(i,t).maneuvers{child_ordinal}.xs(end);
                    trimsInfo(i,t).maneuvers{child_ordinal}.dy = trimsInfo(i,t).maneuvers{child_ordinal}.ys(end);
                    trimsInfo(i,t).maneuvers{child_ordinal}.dyaw = trimsInfo(i,t).maneuvers{child_ordinal}.yaws(end);
    
                    % occupied area of the translated maneuvers
                    [area_x, area_y] = ...
                        translate_global(yaw0,x0,y0,mpa.maneuvers{trim_start,trim_end}.area(1,:),mpa.maneuvers{trim_start,trim_end}.area(2,:));
                    trimsInfo(i,t).maneuvers{child_ordinal}.area = [area_x;area_y];
                    trimsInfo(i,t).maneuvers{child_ordinal}.areaPoly = polyshape(area_x,area_y,'Simplify',false);
    
                    % union of the reachable sets of one parent trim
                    reachable_sets_local{j} = union(reachable_sets_local{j},trimsInfo(i,t).maneuvers{child_ordinal}.areaPoly);
                end
                % union of the reachable sets of all parent trims
                trimsInfo(i,t).reachable_sets = union(trimsInfo(i,t).reachable_sets,reachable_sets_local{j});
                trimsInfo(i,t).reachable_sets_conv = convhull(trimsInfo(i,t).reachable_sets);
            end
            reachable_sets_local{i,t} = trimsInfo(i,t).reachable_sets;
            reachable_sets_conv_local{i,t} = trimsInfo(i,t).reachable_sets_conv;
        end
    end

end

