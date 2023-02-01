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

vehicle = dataByVehicle(~isempty(dataByVehicle));

figure
plot(vehicle.state.create_stamp, vehicle.state.imu_acceleration_forward,'Linewidth',1);
xlabel('$t$ [s]','Interpreter','LaTex') 
ylabel('$a$ [m/s$^2$]','Interpreter','LaTex')

figure
hold on
% Data
plot(vehicle.state.speed,vehicle.state.imu_acceleration_forward,'.');
% limits
manual_mode = ManualMode(111,11);
speed_sorted = sort(vehicle.state.speed);
max_acceleration = manual_mode.compute_max_acceleration(speed_sorted);
min_acceleration = manual_mode.compute_min_acceleration();
plot(speed_sorted,max_acceleration,'--');
plot(speed_sorted,repmat(min_acceleration,1,length(speed_sorted)),'--');
% lables
xlabel('$v$ [m/s]','Interpreter','LaTex') 
ylabel('$a$ [m/s$^2$]','Interpreter','LaTex')

end