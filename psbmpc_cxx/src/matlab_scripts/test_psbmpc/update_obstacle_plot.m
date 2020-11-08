delete(h_X_i{i}); delete(h_X_i_ptch{i}); 

psi_i_k = X_i(3, k);
boat_dim_i_k = rotMatrx2D(psi_i_k)*boat_dim;


% Plot predicted obstacle trajectory
h_X_i{i} = plot(ax1, X_i(2, 1:k),X_i(1, 1:k), 'g', 'Linewidth', 1.6);

% Patch obstacle
h_X_i_ptch{i} = patch(ax1, X_i(2, k)+boat_dim_i_k(2,:),...
    X_i(1, k)+boat_dim_i_k(1,:), 'g', 'LineWidth', 1.6);

% Plot distance to obstacle i
d_0i = sqrt((X(1, 1:k) - X_i(1, 1:k)).^2 +...
    (X(2, 1:k) - X_i(2, 1:k)).^2); 

delete(h_d_i{i}); 
h_d_i{i} = plot(ax2,  d_0i, 'g', 'Linewidth', 1.2);

% Plot 3sigma probability ellipse 
r_ellipse = create_probability_contour(reshape(P_i(:, 1), 4, 4));

delete(h_P_i{i});
h_P_i{i} = plot(ax1, X_i(2, k) + r_ellipse(:, 2), ...
    X_i(1, k) + r_ellipse(:, 1), 'c', 'Linewidth', 1.6); 

legend(ax2, legend_strs, 'Location', 'eastoutside');
drawnow;