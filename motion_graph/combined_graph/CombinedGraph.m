classdef CombinedGraph
    % class for combinedGraph
    
    
    properties
        motionGraphList         % MotionGraph (1 x nVehicles)
        trimTuple               % Matrix with trim indices ((nTrims1*nTrims2*...) x nVehicles)
        transitionMatrix        % binary Matrix (if maneuverTuple exist according to trims) (nTrimTuples x nTrimTuples)
    end
    
    methods
        function obj = CombinedGraph(motionGraphList)
            
            obj.motionGraphList = motionGraphList;
            
            % compute trim tuple (vertices)
            obj.trimTuple = compute_trim_product(obj.motionGraphList);
            
            % compute maneuver matrix for trimProduct
            obj.transitionMatrix = compute_product_maneuver_matrix(obj.motionGraphList);
            
        end
    
        function max_speeds = get_max_speeds(obj)
            % returns maximum speed per vehicle (nVeh x 1)
            nVeh = numel(obj.motionGraphList);
            max_speeds = zeros(nVeh, 1);
            for iVeh = 1:nVeh
                max_speeds(iVeh) = max([obj.motionGraphList(iVeh).trims(:).velocity]);
            end
        end
    end
end

