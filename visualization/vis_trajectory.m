function vis_trajectory(t, x)
figure
plot(x(:,1),x(:,2), '--');
axis equal
hold on


vehicle = patch('XData', zeros(4,1), 'YData', zeros(4,1)); 




for i = 2:numel(t)
    plot(x(1),x(2),'o');
    
    drawnow
end
