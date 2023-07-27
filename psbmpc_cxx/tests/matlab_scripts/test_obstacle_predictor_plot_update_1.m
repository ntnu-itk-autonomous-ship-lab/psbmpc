delete(h_X_i); delete(h_X_i_ptch);

% IF using LOS prediction:
psi_i_k = X_i(3, k);

% ELSE (MROU)
%psi_i_ps_k = atan2(X_i_ps(4, k), X_i_ps(3, k));
%
boat_dim_i_k = rotMatrx2D(psi_i_k) * boat_dim;

% Plot ground truth obstacle trajectory
h_X_i = plot(ax1, X_i(2, 1:k), X_i(1, 1:k), 'k', 'Linewidth', 1.6);

% Patch obstacle
h_X_i_ptch = patch(ax1, X_i(2, k) + boat_dim_i_k(2,:), X_i(1, k) + boat_dim_i_k(1,:), 'b', 'LineWidth', 1.6);

drawnow;