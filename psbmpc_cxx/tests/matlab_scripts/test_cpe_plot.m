figure;
hold on; grid on; axis equal;
plot(v(1, :), 'r');
plot(v(2, :), 'b');
plot(X_i(3, :), 'g');
plot(X_i(4, :), 'c');
legend('$v_{OU,x}$', '$v_{OU,y}$', '$V_x$', '$V_y$', 'Interpreter', 'latex');

figure;
th = 0 : 0.01 : 2.01 * pi;
x_a = 20 * cos(th); y_a = 20 * sin(th);
hold on; grid on;
plot(X(2, :), X(1, :), 'k');
for nwp = 1 : length(WPs(1, :))
    plot(y_a + WPs(2, nwp), x_a + WPs(1, nwp), 'c');
end

plot(WPs(2, :), WPs(1, :), 'rx', 'Linewidth', 1.2);

plot(X_i(2, :), X_i(1, :), 'g');

[~, n_samples] = size(P_flat);
x_cs = 50 * cos(th); y_cs = 50 * sin(th);
for k = 1 : 50 : n_samples
    ell = create_probability_contour(reshape(P_flat(:, k), 4, 4));
    plot(ell(:, 2) + X_i(2, k), ell(:, 1) + X_i(1, k), 'c');
    
    plot(y_cs + X(2, k), x_cs + X(1, k), 'r');
end

legend('OS', 'WPs', 'obst');

figure;
hold on; grid on;
plot(P_c_CE, 'r');
plot(P_c_MCSKF, 'b');
legend('CE', 'MCSKF4D', 'Interpreter', 'latex');

save("pcoll", "P_c_CE", "P_c_MCSKF");
