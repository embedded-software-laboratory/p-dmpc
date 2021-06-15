% MIT License
% 
% Copyright (c) 2021 Lehrstuhl Informatik 11 - RWTH Aachen University
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% 
% This file is part of receding-horizon-graph-search.
% 
% Author: i11 - Embedded Software, RWTH Aachen University

function [ ReferencePoints ] = sampleReferenceTrajectory(nSamples, referenceTrajectory, vehicle_x,vehicle_y, stepSize )
% Computes equidistant points along a piecewise linear curve. The first
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