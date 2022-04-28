function reference = sampleReferenceTrajectory(nSamples, referenceTrajectory, vehicle_x,vehicle_y, stepSize )
% SAMPLEREFERENCETRAJETORY  Computes equidistant points along a piecewise linear curve. The first
% point is the point on the curve closest to the given point
% (vehicle_x,vehicle_y). All following points are on the curve with a
% distance of 'stepSize' to their predecessor.
%
% Arguments: 
%       nSamples: number of points created
%       referenceTrajectory: piecewise linear curve [x1 y1; x2 y2; ...]
%       vehicle_x,vehicle_y: start point
%       stepSize: Distance between points [d1; d2; ...]
% Returns: ReferencePoints [x1 y1; x2 y2; ...] and corresponding
% ReferenceIndex (point index)

    reference = struct;
    reference.ReferencePoints = zeros(nSamples,2);
    reference.ReferenceIndex = zeros(nSamples,1);
    
    [~, ~, x, y, TrajectoryIndex ] = getShortestDistance(referenceTrajectory(:,1),referenceTrajectory(:,2),vehicle_x,vehicle_y);
    
    
    nLinePieces = size(referenceTrajectory,1);
    currentPoint = [x y];
    
    % If the first and the last Point of the reference path are the same(here within a small enough distance),
    % we think the reference is in a loop. 
    Is_refPath_loop = norm((referenceTrajectory(1,:) - referenceTrajectory(end,:)),2) < 1e-8;
    Is_refPoint_last = TrajectoryIndex == nLinePieces;
    TrajectoryIndexLast = TrajectoryIndex - 1;

    % If the refPath is in a loop and the refPoint is the last one, change
    % the TrajectoryIndex to 1
    if Is_refPath_loop && Is_refPoint_last
        TrajectoryIndex = 1; 
    end  

    for i=1:nSamples
        
        remainingLength = norm(currentPoint-referenceTrajectory(TrajectoryIndex,:),2);
        
        if remainingLength > stepSize(i) || TrajectoryIndex == nLinePieces
            %normalize is used to construct a unit vector wrt the line segment
            %if the remaining lenght is larger than step size, then make a
            %step further first until the remaining length is less than the
            %step size
            currentPoint = currentPoint + stepSize(i)*normalize(referenceTrajectory(TrajectoryIndex,:)-referenceTrajectory(TrajectoryIndexLast,:));

        else
            
            while remainingLength < stepSize(i) 
                % while the remaining length is shorter than stepSize,
                % update the currentPoint to the current refPoint and update the
                % current reference point to the next refPoint until the
                % first refPoint after one stepSize is found
                
                reflength = remainingLength;
                currentPoint = referenceTrajectory(TrajectoryIndex,:);
                TrajectoryIndexLast = TrajectoryIndex;
                TrajectoryIndex = min(TrajectoryIndex+1, nLinePieces);
                
                Is_refPoint_last = TrajectoryIndex == nLinePieces;

                if (Is_refPath_loop && Is_refPoint_last)
                    TrajectoryIndex = 1;
                end 
       
                remainingLength = remainingLength + norm(currentPoint-referenceTrajectory(TrajectoryIndex,:),2);
            end
            currentPoint = currentPoint + (stepSize(i)-reflength)*normalize(referenceTrajectory(TrajectoryIndex,:)-referenceTrajectory(TrajectoryIndexLast,:));
%             disp(['trajectoryIndex is :',num2str(TrajectoryIndex)])
        end      
        % record step
        reference.ReferencePoints(i,:) = currentPoint;
        reference.ReferenceIndex(i,:) = TrajectoryIndex;
    end

end


function y=normalize(x)
    y = x/norm(x,2);
end
