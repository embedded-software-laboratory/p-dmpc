%% evaluate reachability analysis
if ~exist('mpa','var')
    load('MPA_trims3_Hp5_parl_non-convex.mat','mpa')
end

% plot trajectory of center point
[nTrims,Hp] = size(mpa.local_reachable_sets_CP);
figure
hold on 
for iTrim = 1
    for t = 2
        CP = mpa.local_reachable_sets_CP{iTrim,t};
        for j = 1:length(CP)
            if all(CP{j}(2,:)==0)
                LineWidth = 2;
                LineStyle = '--';
            else
                LineWidth = 0.5;
                LineStyle = '-';
            end
            plot(CP{j}(1,:),CP{j}(2,:),'LineWidth',LineWidth,'LineStyle',LineStyle)
        end
    end
end
axis equal
xlim([0,0.6])
ylim([-0.3,0.3])

%% compare the computation time with CORA toolbox
x0 = 0.1; y0 = 0.2; yaw0 = pi/9;
iTrim = 1;
[nTrims,Hp] = size(mpa.local_reachable_sets_CP);
% translate local reachable sets to global
figure 
hold on
iters = 1;

start_1 = tic;
for i = 1:iters
    for step = 1:Hp-3
        [x_local,y_local] = mpa.local_reachable_sets_CP{iTrim,step}.boundary;
        [x_global,y_global] = translate_global(yaw0,x0,y0,x_local',y_local');
%         plot(x_global,y_global)
        % plot center trajectory
        for j = 1:length(mpa.local_center_trajectory{iTrim,step})
%             if j<=4 
                [x_global_CP,y_global_CP] = translate_global(yaw0,x0,y0,mpa.local_center_trajectory{iTrim,step}{j}(1,:),mpa.local_center_trajectory{iTrim,step}{j}(2,:));
                plot(x_global_CP,y_global_CP)
%             end
        end
    end
end
time_1 = toc(start_1);
disp(['Mean computation of the appraoch with offline and online reachability analisis: ' num2str(time_1/iters) ' seconds.'])

% single track bicycle model
f_bicycle = @(x, u) [u(2)*cos(x(3)+atan(0.5*tan(u(1))));
             u(2)*sin(x(3)+atan(0.5*tan(u(1))));
             u(2)/0.2*tan(u(1))*cos(atan(0.5*tan(u(1))))];

% nonlinear system
numStates = 3;
numInputs = 2;
sys = nonlinearSys(f_bicycle,numStates,numInputs);

% reachability settings
options.timeStep = 0.2;
options.zonotopeOrder = 10;
options.taylorTerms = 5;
options.alg = 'lin';
options.tensorOrder = 2;

% parameter
Lf = 0.1; Lr = 0.1; Width = 0.1;
params.tFinal = step*options.timeStep;
x0_lower = [x0;y0;yaw0]; % [x0;y0;yaw0]
x0_upper = [x0;y0;yaw0]; 
steering_min = min([mpa.trims.steering]);
steering_max = max([mpa.trims.steering]);
% steering_min = 0;
% steering_max = 0;
speed_min = min([mpa.trims.speed]);
speed_max = max([mpa.trims.speed]);
u0_lower = [steering_min;speed_min]; % [steering; speed]
u0_upper = [steering_max;speed_max];
params.R0 = zonotope(interval(x0_lower,x0_upper));
params.U = zonotope(interval(u0_lower,u0_upper));

% reachability analysis

start_2 = tic;
for i = 1:iters
    R = reach(sys,params,options);
end
time_2 = toc(start_2);
disp(['Mean computation of pure online appraoch (CORA toolbox): ' num2str(time_2/iters) ' seconds.'])
disp(['Reduce the computation time by ' num2str(1-time_1/time_2) '%.'])
plot(R);
hold on 
% patch([-Lr,-Lr,Lf,Lf],[-Width/2,Width/2,Width/2,-Width/2],'r')
axis equal
