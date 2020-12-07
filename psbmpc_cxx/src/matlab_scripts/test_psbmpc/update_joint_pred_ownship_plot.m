delete(h_X); delete(h_safe); delete(h_X_ptch); 
boat_dim_l = rotMatrx2D(X(3, k))*boat_dim;
h_X = plot(ax1, X(2, 1:k), X(1, 1:k),'k', 'Linewidth', 1.6);

%h_traj_text = text(ax1, X(2, k) + 5, X(1, k), 'OS')
h_safe = plot(ax1, d_safe * y_cs + X(2, k),  d_safe * x_cs + X(1, k), 'r');

h_X_ptch = patch(ax1, X(2, k)+boat_dim_l(2,:),X(1, k)+boat_dim_l(1,:),'k', 'Linewidth', 1.6);