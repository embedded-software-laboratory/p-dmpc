n = 5e2;
nVeh = 5;

%% node array
depth = 0;
trims = ones(nVeh,1);
xs = ones(nVeh,1);
ys = ones(nVeh,1);
yaws = ones(nVeh,1);
g_values = ones(nVeh,1);
h_values = zeros(nVeh,1);
myTree(n) = node(depth, trims, xs, ys, yaws, g_values, h_values);

% for ii = 1:n
%     myTree = change_node(myTree, ii);
% end

for ii = randperm(n)
    myTree = change_node(myTree, ii);
end

for ii = randperm(n)
    myTree(ii).ys = 10*rand(size(myTree(ii).xs));
end


%% matrix



function t = change_node(t, iNode)    
t(iNode).xs = 10*rand(size(t(iNode).xs));
end