%% Four velocities (0 to 4 m/s), one steering (0 deg)
u_trims = zeros(4,2);
u_trims(:,2) = (0:3);
trim_adjacency = [1 1 0 0; 1 1 1 0; 0 1 1 1; 0 0 1 1];
save('trim_library/mpa_1_st_4_vel', 'u_trims', 'trim_adjacency');