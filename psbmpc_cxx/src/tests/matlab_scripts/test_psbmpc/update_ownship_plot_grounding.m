

delete(h_X); delete(h_X_p); delete(h_safe); delete(h_X_ptch); 
dim_X = size(X);

boat_dim_l = rotMatrx2D(X(3, k))*boat_dim;
%fprintf('X(3, k) = %1.3f\n', (180/ pi) * X(3, k));

h_X = plot(ax1, map_origin(2) + X(2, 1:k), map_origin(1) + X(1, 1:k),'k', 'Linewidth', 1.6);

h_X_p = plot(ax1, map_origin(2) + X_pred(2, :), map_origin(1) + X_pred(1, :),'b', 'Linewidth', 1.6);
%h_traj_text = text(ax1, xs_p(k,2) + 5, xs_p(k,1), 'OS')
h_safe = plot(ax1, d_safe * y_cs + map_origin(2) + X(2, k),  d_safe * x_cs + map_origin(1) + X(1, k), 'r');

h_X_ptch = patch(ax1, map_origin(2) + X(2, k) + boat_dim_l(2, :), map_origin(1) + X(1, k) + boat_dim_l(1, :),'k', 'Linewidth', 1.6);
