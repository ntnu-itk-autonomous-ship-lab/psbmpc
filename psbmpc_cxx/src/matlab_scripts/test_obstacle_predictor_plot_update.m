delete(h_X_i_ps{ps}); delete(h_X_i_ps_ptch{ps}); 

psi_i_ps_k = X_i_ps(3, k);
boat_dim_i_ps_k = rotMatrx2D(psi_i_ps_k) * boat_dim;


% Plot ground truth obstacle trajectory
h_X_i_ps{ps} = plot(ax1, X_i_ps(2, 1:k), X_i_ps(1, 1:k), 'g', 'Linewidth', 1.6);

% Patch obstacle
h_X_i_ps_ptch{ps} = patch(ax1, X_i_ps(2, k)+boat_dim_i_ps_k(2,:),...
    X_i_ps(1, k)+boat_dim_i_ps_k(1,:), 'g', 'LineWidth', 1.6);

% Plot 3sigma probability ellipse 
r_ellipse = create_probability_contour(reshape(P_i_ps(:, k), 4, 4));

delete(h_P_i_ps{ps});
h_P_i_ps{ps} = plot(ax1, X_i(2, k) + r_ellipse(:, 2), ...
    X_i(1, k) + r_ellipse(:, 1), 'c', 'Linewidth', 1.6); 

% Plot velocity components against mean mrou velocity components
delete(h_gt_Vx_i_ps{ps}); delete(h_mean_Vx_i_ps{ps}); 
delete(h_gt_Vy_i_ps{ps}); delete(h_mean_Vy_i_ps{ps});
h_gt_Vx_i_ps{ps} = plot(ax2, t_vec(1:k), X_i_ps(4, 1:k) .* cos(X_i(3, 1:k)), 'Linewidth', 1.6);
h_mean_Vx_i_ps{ps} = plot(ax2, t_vec(1:k), v_i_ps(1, 1:k), 'Linewidth', 1.6);

h_gt_Vy_i_ps{ps} = plot(ax6, t_vec(1:k), X_i_ps(4, 1:k) .* sin(X_i_ps(3, 1:k)), 'Linewidth', 1.6);
h_mean_Vy_i_ps{ps} = plot(ax6, t_vec(1:k), v_i_ps(2, 1:k), 'Linewidth', 1.6);


legend(ax2, 'Vx', 'mean Vx', 'Location', 'northeast');
legend(ax2, legend_strs, 'Location', 'northeast');

legend(ax3, 'Vy', 'mean Vy', 'Location', 'northeast');
legend(ax3, legend_strs, 'Location', 'northeast');

drawnow;