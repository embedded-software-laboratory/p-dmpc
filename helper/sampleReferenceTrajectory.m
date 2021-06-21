function [ ReferencePoints ] = sampleReferenceTrajectory(nSamples, referenceTrajectory, vehicle_x,vehicle_y, stepSize )
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
% Returns: points [x1 y1; x2 y2; ...]

    ReferencePoints = zeros(nSamples,2);
    
    [~, ~, x, y, TrajectoryIndex ] = getShortestDistance(referenceTrajectory(:,1),referenceTrajectory(:,2),vehicle_x,vehicle_y);
    
    nLinePieces = size(referenceTrajectory,1);
    currentPoint = [x y];
    
    % All line-segments are assumed to be longer than stepSize. Should it
    % become necessary to have short line-segments this algorithm needs to
    % be changed.
    for i=1:nLinePieces-1
        assert(norm(referenceTrajectory(i+1,:)-referenceTrajectory(i,:),2)>max(stepSize));
    end
    
    for i=1:nSamples
        % make a step
        remainingLength = norm(currentPoint-referenceTrajectory(TrajectoryIndex,:),2);
        if remainingLength > stepSize(i) || TrajectoryIndex == nLinePieces
            currentPoint = currentPoint + stepSize(i)*normalize(referenceTrajectory(TrajectoryIndex,:)-referenceTrajectory(TrajectoryIndex-1,:));
        else
            currentPoint = referenceTrajectory(TrajectoryIndex,:);            
            TrajectoryIndex = min(TrajectoryIndex+1, nLinePieces);            
            currentPoint = currentPoint + (stepSize(i)-remainingLength)*normalize(referenceTrajectory(TrajectoryIndex,:)-referenceTrajectory(TrajectoryIndex-1,:));
        end
        
        % record step
        ReferencePoints(i,:) = currentPoint;
    end
end


function y=normalize(x)
    y = x/norm(x,2);
end