
plot(ax1, X_i(2, :), X_i(1, :), 'g', 'Linewidth', 1.2);
[~, n_samples] = size(X_i);
for s = 1 : 100 : n_samples
    
    text(ax1, X_i(2, s) - 70, X_i(1, s) + 20, ['o,ps,k=', num2str((s-1) * 0.5)]);
    plot(ax1, X_i(2, s), X_i(1, s), 'go', 'Linewidth', 1.6);
    
    ell = create_probability_contour(reshape(P_i_flat(:, s), 4, 4));
    plot(ax1, ell(:, 2) + X_i(2, s), ell(:, 1) + X_i(1, s), 'c');
    
    
end

