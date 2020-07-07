classdef BicycleModelConstSpeed < VehicleModel
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Lf
        Lr
    end
    
    methods
        function obj = BicycleModelConstSpeed(Lf, Lr)
            %UNTITLED5 Construct an instance of this class
            %   Detailed explanation goes here
            obj.nx = 4;
            obj.nu = 1;
            obj.ny = 2;
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
            % From Vehicle Dynamics and Control, Rajesh Rajamani, p. 24
            
            L = obj.Lf+obj.Lr;
            R = obj.Lr/L;
            psi = x(3);
            v_center = x(4);
            beta = atan(R*tan(u));

            dx = x;
            dx(1) = v_center*cos(psi+beta);
            dx(2) = v_center*sin(psi+beta);
            dx(3) = v_center/L*tan(u)*cos(beta);
            dx(4) = 0;
        end
    end
end
