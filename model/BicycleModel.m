classdef BicycleModel < VehicleModel
    
    properties
        Lf
        Lr
        W
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
            
            % width
            obj.W = 0.88;
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
