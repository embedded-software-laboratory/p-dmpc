function test_bicycle_model
    model = BicycleModel(2.2, 2.2);
    dt = 30;
    u = [-0.6, 0, 0.6];
    x0 = zeros(model.nx, 1);
    yaw0 = 0;
    speed0 = 3;
    x0(3) = yaw0; 
    x0(4) = speed0; 

for this_u = u
[t, x] = ode45(@(t, x) model.ode(x, this_u), ...
               [0 dt], ...
               x0, ...
               odeset('RelTol',1e-8,'AbsTol',1e-8));
figure
plot(x(:,1),x(:,2));
axis equal
hold on
% plot
end
end
