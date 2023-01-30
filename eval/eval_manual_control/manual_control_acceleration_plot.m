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
title('Acceleration','Interpreter','LaTex')

figure
plot(vehicle.state.speed,vehicle.state.imu_acceleration_forward,'.');
xlabel('$v$ [m/s]','Interpreter','LaTex') 
ylabel('$a$ [m/s$^2$]','Interpreter','LaTex')
title('Acceleration','Interpreter','LaTex')
end