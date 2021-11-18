delete(h_X); delete(h_safe); delete(h_X_ptch); 

boat_dim_l = rotMatrx2D(X(3, k))*boat_dim;
h_X = plot(ax1, X(2, 1:k), X(1, 1:k),'k', 'Linewidth', 1.6);

%h_traj_text = text(ax1, xs_p(k,2) + 5, xs_p(k,1), 'OS')
h_safe = plot(ax1, d_safe * y_cs + X(2, k),  d_safe * x_cs + X(1, k), 'r', 'Linewidth', 1.6);

h_X_ptch = patch(ax1, X(2, k)+boat_dim_l(2,:),X(1, k)+boat_dim_l(1,:),'k', 'Linewidth', 1.6);

for s = 1 : t_ts / dt_sim : n_samples
    delete(h_X_text_s{s}); delete(h_safe_s{s}); delete(h_X_s{s});
    h_X_text_s{s} = text(ax1, X(2, s) - 70, X(1, s) + 20, ['OS, t=', num2str((s-1) * dt_sim)]);
    
    boat_dim_s = rotMatrx2D(X(3, s))*boat_dim;
    h_X_s{s} = patch(ax1, X(2, s)+boat_dim_s(2,:),X(1, s)+boat_dim_s(1,:),'k', 'Linewidth', 1.6);
    
    h_safe_s{s} = plot(ax1, y_cs * d_safe + X(2, s), d_safe * x_cs + X(1, s), 'r', 'Linewidth', 1.6);
end
