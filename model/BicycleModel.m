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

classdef BicycleModel < VehicleModel
    
    properties
        Lf
        Lr
    end
    
    methods
        function obj = BicycleModel(Lf, Lr)
            % noe state vector
            obj.nx = 5;
            
            % noe input vector
            obj.nu = 2;
            
            % noe properties model
            obj.ny = 3;

            % lengths from center of gravity
            obj.Lf = Lf;
            obj.Lr = Lr;
        end
        
        
        function dx = ode(obj, x, u)
            %ode Discrete step in centered bicycle model
            %   x is 
            %       x position
            %       y position
            %       yaw
            %       speed
            %       steering
            %   u is
            %       steering derivative
            %       acceleration
            % From Vehicle Dynamics and Control, Rajesh Rajamani, p. 24
            
            L = obj.Lf+obj.Lr;
            R = obj.Lr/L;
            psi = x(3);
            v_center = x(4);
            delta = x(5);
            steering_derivative = u(1);
            acceleration = u(2);
            beta = atan(R*tan(delta));

            dx = x;
            dx(1) = v_center*cos(psi+beta);
            dx(2) = v_center*sin(psi+beta);
            dx(3) = v_center/L*tan(delta)*cos(beta);
            dx(4) = acceleration;
            dx(5) = steering_derivative;
        end
    end
end
