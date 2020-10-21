hold on;
disp(X_i(:, 301)');
plot(gca, X_i(2, :), X_i(1, :), 'g', 'Linewidth', 1.2);

x_cs = 50 * cos(th); y_cs = 50 * sin(th);
for k = 1 : 100 : n_samples
    
    text(X(2, k) - 70, X(1, k) + 20, ['OS, t = ', num2str((k-1) * 0.5)]);
    plot(X(2, k) , X(1, k), 'ko', 'Linewidth', 1.6);
    text(X_i(2, k) - 70, X_i(1, k) + 20, ['o, ps, k = ', num2str((k-1) * 0.5)]);
    plot(X_i(2, k), X_i(1, k), 'go', 'Linewidth', 1.6);
    ell = create_probability_contour(reshape(P_i_flat(:, k), 4, 4));
    plot(gca, ell(:, 2) + X_i(2, k), ell(:, 1) + X_i(1, k), 'c');
    
    plot(gca, y_cs + X(2, k), x_cs + X(1, k), 'r');
end
