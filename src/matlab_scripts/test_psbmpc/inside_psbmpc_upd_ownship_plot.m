

delete(h_X); delete(h_X_k); delete(h_X_text); delete(h_safe); delete(h_safe_k); delete(h_X_ptch); 
boat_dim_l = rotMatrx2D(X(3, k))*boat_dim;
h_X = plot(ax1, X(2, 1:k), X(1, 1:k),'k', 'Linewidth', 1.6);

%h_traj_text = text(ax1, xs_p(k,2) + 5, xs_p(k,1), 'OS')
h_safe = plot(ax1, d_safe * y_cs + X(2, k),  d_safe * x_cs + X(1, k), 'r');

h_X_ptch = patch(ax1, X(2, k)+boat_dim_l(2,:),X(1, k)+boat_dim_l(1,:),'k', 'Linewidth', 1.6);

for s = 1 : 125 : n_samples
    h_X_text = text(ax1, X(2, s) - 70, X(1, s) + 20, ['OS, t=', num2str((s-1) * 0.5)]);
    h_X_k = plot(ax1, X(2, s) , X(1, s), 'ko', 'Linewidth', 1.6);
    h_safe_k = plot(ax1, y_cs * 50 + X(2, s), 50 * x_cs + X(1, s), 'r', 'Linewidth', 1.6);
end

drawnow;
