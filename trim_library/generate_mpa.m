%% Four velocities (0 to 4 m/s), one steering (0 deg)
trim_inputs = zeros(4,2);
trim_inputs(:,2) = (0:3);
trim_adjacency = [1 1 0 0; 1 1 1 0; 0 1 1 1; 0 0 1 1];
save('trim_library/mpa_1_st_4_vel', 'trim_inputs', 'trim_adjacency');