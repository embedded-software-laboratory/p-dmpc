function vis_trajectory(paths, target_poses, axis_size, varargin)

    if nargin==1
        fig = figure;
    else
        fig = varargin{1};
    end
    
    n_veh = length(paths(1,1,:));
    height = length(paths(:,1,1));
    
    vehColors = [0.8941    0.1020    0.1098;...
                 0.2157    0.4941    0.7216;...
                 0.3020    0.6863    0.2902;...
                 0.5961    0.3059    0.6392;...
                 1.0000    0.4980    0     ;...
                 1.0000    1.0000    0.2000;...
                 0.6510    0.3373    0.1569;...
                 0.9686    0.5059    0.7490];
    
    plots = gobjects(1, n_veh);
    descriptions = strings(1, n_veh); 
                    
    
    
    for i = 1:n_veh
        
        hold on
        plots(i) = plot(paths(:,1,i),paths(:,2,i), '--', 'Color', vehColors(mod(i-1,size(vehColors,1))+1,:));
        descriptions(i) = 'Vehicle: ' + string(i);
        axis(axis_size);
        
        radius = 1;
        center = [target_poses(i).x, target_poses(i).y];
        position = [center - radius, 2*radius, 2*radius];
        rectangle('Position',position,'Curvature',[1 1], 'EdgeColor', vehColors(mod(i-1,size(vehColors,1))+1,:))
                
    end

    legend(plots, descriptions);
    
end