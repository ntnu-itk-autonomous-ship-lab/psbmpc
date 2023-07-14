[~, n_samples] = size(P_flat);
fprintf('P(1) = %.2f', reshape(P_flat(:, 1), 4, 4));
figure; axis equal
hold on; grid on;
plot(X(2, :), X(1, :), 'k');
for k = 1 : 50 : n_samples
    ell = create_probability_contour(reshape(P_flat(:, k), 4, 4));
    plot(ell(:, 2) + X(2, k), ell(:, 1) + X(1, k), 'c');
end

figure;
hold on; grid on;
plot(v(1, :), 'r');
plot(v(2, :), 'b');
plot(X(3, :), 'g');
plot(X(4, :), 'c');
legend('$v_{OU,x}$', '$v_{OU,y}$', '$V_x$', '$V_y$', 'Interpreter', 'latex');