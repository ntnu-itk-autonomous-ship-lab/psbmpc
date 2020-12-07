delete(h_X_i{i}); delete(h_X_i_ptch{i}); delete(h_X_i_p{i}); delete(h_text_i{i}); delete(h_safe_i{i});

psi_i_k = atan2(X_i(4, k), X_i(3, k));
boat_dim_i_k = rotMatrx2D(psi_i_k)*boat_dim;

% Plot predicted obstacle trajectory
h_X_i{i} = plot(ax1, X_i(2, 1:k),X_i(1, 1:k), 'g', 'Linewidth', 1.6);

% Plot predicted obstacle trajectory from Obstacle_SBMPC
h_X_i_p{i} = plot(ax1, X_i_pred(2, :),X_i_pred(1, :), '--g', 'Linewidth', 1.6);

% Patch obstacle
h_X_i_ptch{i} = patch(ax1, X_i(2, k)+boat_dim_i_k(2,:),...
    X_i(1, k)+boat_dim_i_k(1,:), 'g', 'LineWidth', 1.6);

% % Plot distance to obstacle i
% d_0i = sqrt((X(1, 1:k) - X_i(1, 1:k)).^2 +...
%     (X(2, 1:k) - X_i(2, 1:k)).^2); 
% 
% delete(h_d_i{i}); 
% h_d_i{i} = plot(ax2,  d_0i, 'g', 'Linewidth', 1.2);

drawnow;