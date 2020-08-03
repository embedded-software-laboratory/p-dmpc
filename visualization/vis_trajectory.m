function vis_trajectory(path)
figure
plot(path(:,1),path(:,2), '--');
axis equal
hold on

X = [0 0 0.4 0.4];
Y = [0.8 0 0 0.8];

car = polyshape(X,Y);
car = rotate(car,-rad2deg(path(2,3)));

plot(car);
end
