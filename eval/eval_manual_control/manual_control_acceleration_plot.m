function manual_control_acceleration_plot()
lcc_vis_folder_path = '../../lab_control_center/recording/visualization';
assert(isfolder(lcc_vis_folder_path))
addpath(lcc_vis_folder_path)

dds_domain = getenv('DDS_DOMAIN');
recording_folders = dir('/tmp/cpm_lab_recordings/*');
current_folder = recording_folders(end);
assert(current_folder.isdir);
recording_file = fullfile(...
    current_folder.folder, ...
    current_folder.name, ...
    'recording.dat' ...
);

dataByVehicle = preprocessing(dds_domain, recording_file);


vehicle = dataByVehicle(end);

% dspeed = diff(vehicle.state.speed);
% dt = diff(vehicle.state.create_stamp);
% acceleration = dspeed./dt;

figure
hold on
plot(vehicle.state.create_stamp, vehicle.state.imu_acceleration_forward,'Linewidth',1);
% plot(vehicle.state.create_stamp,[0; acceleration]);
xlabel('$t$ [s]','Interpreter','LaTex') 
ylabel('$a$ [m/s$^2$]','Interpreter','LaTex')

fig = figure;
hold on

% Data
plot(vehicle.state.speed,vehicle.state.imu_acceleration_forward);
% plot(vehicle.state.speed,[0; acceleration]);
% limits
speed_sorted = sort(vehicle.state.speed);
max_acceleration = ManualMode.compute_max_acceleration(speed_sorted);
min_acceleration = ManualMode.compute_min_acceleration();
color_order = rwth_color_order;
plot(speed_sorted,max_acceleration,'--','Color',color_order(5,:));
plot(speed_sorted,repmat(min_acceleration,1,length(speed_sorted)),'--','Color',color_order(5,:));
% lables
xlabel('$v$ [m/s]','Interpreter','LaTex') 
ylabel('$a$ [m/s$^2$]','Interpreter','LaTex')
set_figure_properties(fig,ExportFigConfig.paper);

filepath = fullfile('.','results','hdv_acceleration.pdf');
export_fig(fig,filepath);

end