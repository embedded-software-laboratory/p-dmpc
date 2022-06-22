function color = vehColor(index,priority_list)
% VEHCOLOR  Return vehicle color specifier for vehicle with given index.

    if nargin==2
%         nVeh = length(priority_list);
%         CM = jet(nVeh);
%         color = CM(index,:);
        vehColors = [1.0  0.0  0.0; % red
                     1.0  0.5  0.5; %   light red
                     1.0  0.0  1.0; % purple
                     1.0  0.5  1.0; %   light purple
                     0.0  0.0  1.0; % blue
                     0.5  0.5  1.0; %   light blue
                     0.0  1.0  0.0; % green
                     0.5  1.0  0.5; %   light green
                     1.0  1.0  0.0; % yellow
                     1.0  1.0  0.5; %   light yellow
                     1.0  1.0  1.0]; % white
        nPriorities = length(unique(priority_list));

        
        n_threshold = 4; % minimum number of colors
        nColors = max(nPriorities,n_threshold);
        ys = linspace(0,1,nColors)';
        zs = linspace(0,1,nColors)';
%         vehColors = [ones(nColors,1),ys,zs];
        CM = hot(max(nPriorities,n_threshold));
        
        pri = priority_list(index);
        
%         color = CM(pri,:);
        color = vehColors(pri,:);
    else
        vehColors = [0.8941    0.1020    0.1098;...
                     0.2157    0.4941    0.7216;...
                     0.3020    0.6863    0.2902;...
                     0.5961    0.3059    0.6392;...
                     1.0000    0.4980    0     ;...
                     1.0000    1.0000    0.2000;...
                     0.6510    0.3373    0.1569;...
                     0.9686    0.5059    0.7490];
        color = vehColors(mod(index-1,size(vehColors,1))+1,:);
    end
end
