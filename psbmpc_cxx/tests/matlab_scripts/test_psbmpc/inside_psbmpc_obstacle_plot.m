delete(h_X_i{i, ps});
delete(h_X_i_s{i, ps});
delete(h_X_i_ptch{i, ps});
delete(h_P_i{i, ps});
delete(h_text_i{i, ps});

h_X_i{i, ps} = plot(ax1, X_i(2, :), X_i(1, :), 'g', 'Linewidth', 1.2);

chi_i_ps = atan2(X_i(4, end), X_i(3, end));
boat_dim_end = rotMatrx2D(chi_i_ps) * boat_dim;
h_X_i_ptch{i, ps} = patch(ax1, X_i(2, end) + boat_dim_end(2,:),X_i(1, end)+boat_dim_end(1,:),'g', 'Linewidth', 1.6);
h_text_i{i, ps} = text(ax1, X_i(2, end) - 10, X_i(1, end) + 10, ['i=', num2str(i-1), ', ps=', num2str(ps-1)]);

ell = create_probability_contour(reshape(P_i_flat(:, end), 4, 4));
h_P_i{i, ps} = plot(ax1, ell(:, 2) + X_i(2, end), ell(:, 1) + X_i(1, end), 'c');

for s = 1 : n_samples / n_patches : n_samples
    
    
       
    chi_i_ps = atan2(X_i(4, s), X_i(3, s));
    boat_dim_s = rotMatrx2D(chi_i_ps)*boat_dim;
    h_X_i_s{i, ps} = patch(ax1, X_i(2, s)+boat_dim_s(2,:),X_i(1, s)+boat_dim_s(1,:),'g', 'Linewidth', 1.6)
    
    ell = create_probability_contour(reshape(P_i_flat(:, s), 4, 4));
    h_P_i{i, ps} = plot(ax1, ell(:, 2) + X_i(2, s), ell(:, 1) + X_i(1, s), 'c');
end
drawnow;
