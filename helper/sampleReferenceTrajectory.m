function [ ReferencePoints ] = sampleReferenceTrajectory(scenarioName,nSamples, referenceTrajectory, vehicle_x,vehicle_y, stepSize )
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
    
    if ~strcmp(scenarioName, 'Commonroad')   
    
        for i=1:nSamples
            % make a step
            remainingLength = norm(currentPoint-referenceTrajectory(TrajectoryIndex,:),2);
            if remainingLength > stepSize(i) || TrajectoryIndex == nLinePieces
                currentPoint = currentPoint + stepSize(i)*normalize(referenceTrajectory(TrajectoryIndex,:)-referenceTrajectory(TrajectoryIndex-1,:));
            else

                while remainingLength < stepSize(i) % works for line-segments shorter than the stepSize
                    reflength = remainingLength;
                    currentPoint = referenceTrajectory(TrajectoryIndex,:);
                    TrajectoryIndex = min(TrajectoryIndex+1, nLinePieces);
                    remainingLength = remainingLength + norm(currentPoint-referenceTrajectory(TrajectoryIndex,:),2);
                end
                currentPoint = currentPoint + (stepSize(i)-reflength)*normalize(referenceTrajectory(TrajectoryIndex,:)-referenceTrajectory(TrajectoryIndex-1,:));
            end      
            % record step
            ReferencePoints(i,:) = currentPoint;
        end
    
    else
        %% referenth path in a loop
        % the last reference point is the first reference point
        if TrajectoryIndex == nLinePieces
            TrajectoryIndex = 1;
            TrajectoryIndexLast = nLinePieces - 1;
        else
            TrajectoryIndexLast = TrajectoryIndex - 1;
        end

        for i=1:nSamples
            % make a step
            remainingLength = norm(currentPoint-referenceTrajectory(TrajectoryIndex,:),2);
            if remainingLength > stepSize(i)

                disp(['Trajectory Index * is: ', num2str(TrajectoryIndex)])
                currentPoint = currentPoint + stepSize(i)*normalize(referenceTrajectory(TrajectoryIndex,:)-referenceTrajectory(TrajectoryIndexLast,:));
            else
    %             currentPoint = referenceTrajectory(TrajectoryIndex,:);
    %             TrajectoryIndex = min(TrajectoryIndex+1, nLinePieces);
    %             currentPoint = currentPoint + (stepSize(i)-remainingLength)*normalize(referenceTrajectory(TrajectoryIndex,:)-referenceTrajectory(TrajectoryIndex-1,:));

                while remainingLength < stepSize(i) % works for line-segments shorter than the stepSize
                    reflength = remainingLength;
                    currentPoint = referenceTrajectory(TrajectoryIndex,:);
                    TrajectoryIndexLast = TrajectoryIndex;
                    TrajectoryIndex = min(TrajectoryIndex+1, nLinePieces);

                    if TrajectoryIndex == nLinePieces
                        TrajectoryIndex = 1;
                    end

                    remainingLength = remainingLength + norm(currentPoint-referenceTrajectory(TrajectoryIndex,:),2);
                end
                currentPoint = currentPoint + (stepSize(i)-reflength)*normalize(referenceTrajectory(TrajectoryIndex,:)-referenceTrajectory(TrajectoryIndexLast,:));
            disp(['Trajectory Index is: ', num2str(TrajectoryIndex)])
            end

            % record step
            ReferencePoints(i,:) = currentPoint;
            disp(['ReferencePoint',num2str(i) ' is: ', num2str(currentPoint)])
        end

    end
end


function y=normalize(x)
    y = x/norm(x,2);
end
