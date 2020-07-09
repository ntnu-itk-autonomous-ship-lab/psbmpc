persistent ps_counter;
if isempty(ps_counter)
    ps_counter = 1;
else
    ps_counter = ps_counter + 1;
end
plot(X_i(2, :), X_i(1, :), 'g');

x_cs = 50 * cos(th); y_cs = 50 * sin(th);
for k = 1 : 50 : n_samples
    ell = create_probability_contour(reshape(P_flat(:, k), 4, 4));
    plot(ell(:, 2) + X_i(2, k), ell(:, 1) + X_i(1, k), 'c');
    
    plot(y_cs + X(2, k), x_cs + X(1, k), 'r');
end
legend('OS', 'WPs', ['(i, ps) = (1, ',  num2str(ps_counter), ')']);

figure;
hold on; grid on; axis equal;
subplot(311);
plot(X_i(3, :), 'b');
ylabel('Obstacle north speed');
subplot(312);
plot(X_i(4, :), 'b');
ylabel('Obstacle east speed');
subplot(313);
plot((180 / pi) * atan2(X_i(4, :), X_i(3, :)), 'b');
ylabel('Obstacle course');
