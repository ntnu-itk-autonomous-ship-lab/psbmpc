delete(h_X_i{i, ps});
delete(h_X_i_s{i, ps});
delete(h_X_i_ptch{i, ps});
delete(h_P_i{i, ps});
delete(h_text_i{i, ps});

h_X_i{i, ps} = plot(ax1, X_i(2, :), X_i(1, :), 'g', 'Linewidth', 1.2);

chi_i_ps = atan2(X_i(4, end), X_i(3, end));
boat_dim_end = rotMatrx2D(chi_i_ps)*boat_dim;
h_X_i_ptch{i, ps} = patch(ax1, X_i(2, end)+boat_dim_end(2,:),X_i(1, end)+boat_dim_end(1,:),'g', 'Linewidth', 1.6);

[~, n_samples] = size(X_i);
for s = 1 : 100 : n_samples
    
    h_text_i{i, ps} = text(ax1, X_i(2, s) - 70, X_i(1, s) + 20, ['o,ps,k=', num2str((s-1) * 0.5)]);
    h_X_i_s{i, ps} = plot(ax1, X_i(2, s), X_i(1, s), 'go', 'Linewidth', 1.6);
    
    ell = create_probability_contour(reshape(P_i_flat(:, s), 4, 4));
    h_P_i{i, ps} = plot(ax1, ell(:, 2) + X_i(2, s), ell(:, 1) + X_i(1, s), 'c');
end
drawnow;
