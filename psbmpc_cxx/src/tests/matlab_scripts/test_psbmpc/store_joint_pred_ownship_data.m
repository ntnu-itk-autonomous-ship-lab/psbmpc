
% Create ownship data structure
os.X = X;
os.u_d = zeros(1, n_samples);
os.u_m = zeros(1, n_samples);
os.chi_d = zeros(1, n_samples);
os.chi_m = zeros(1, n_samples);
os.dXdt = zeros(6, n_samples);
os.tau = zeros(3, n_samples);
os.e = zeros(1, n_samples);
os.P_coll = zeros(1, n_samples);

save(['joint_pred_data'], 'os');
