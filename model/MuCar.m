classdef MuCar < VehicleModel
    properties
        p = [1.004582, -0.142938, 0.195236, 3.560576, -2.190728, -9.726828, 2.515565, 1.321199, 0.032208, -0.012863]; % parameter vector
    end

    methods
        function obj = MuCar()
            obj.nx = 4;
            obj.nu = 2;
        end

        function dx = ode(obj,x,u,t,t_u)
            % See vehicle paper https://doi.org/10.1016/j.ifacol.2020.12.1821
            u = interp1(t_u,u,t, 'method', 'previous');
            
            px    = x(:,1);
            py    = x(:,2);
            yaw   = x(:,3);
            v     = x(:,4);
            
            f          = u(:,1);
            delta_ref  = u(:,2);
            
            delta = delta_ref + obj.p(8);
            
            dx = 0*x;
            
            dx(:,1) = obj.p(1) .* v .* (1 + obj.p(2) .* delta.^2) .* cos(yaw + obj.p(3) .* delta + obj.p(9));
            dx(:,2) = obj.p(1) .* v .* (1 + obj.p(2) .* delta.^2) .* sin(yaw + obj.p(3) .* delta + obj.p(9));
            dx(:,3) = obj.p(4) .* v .* delta;
            dx(:,4) = obj.p(5) .* v + obj.p(6) .* sign(f) .* (abs(f).^(obj.p(7)));
        end

        
        function u = compute_input_from_trim(obj,trim_in)
            % 
            u = zeros(obj.nu,1);
            u(1) = sign(trim_in.speed) * nthroot(obj.p(5)/obj.p(6) .* trim_in.speed, obj.p(7));
            u(2) = steering - obj.p(8);
        end
    end

end