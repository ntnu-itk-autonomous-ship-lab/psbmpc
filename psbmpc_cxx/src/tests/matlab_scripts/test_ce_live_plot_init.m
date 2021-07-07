
figure; grid on; axis equal
hold on;
xlabel('East'); ylabel('North');
ax = gca;
th = 0 : 0.01 : 2.1 * pi;
x_cs = 50 * cos(th); y_cs = 50 * sin(th);
plot(ax, p_os(2), p_os(1), 'ro', 'Linewidth', 2);
plot(ax, p_i(2), p_i(1), 'bo', 'Linewidth', 2);

plot(ax, p_os(2) + y_cs, p_os(1) + x_cs, 'r', 'Linewidth', 2);

i_ell = create_probability_contour(P_i);
plot(ax, p_i(2) + i_ell(:, 2), p_i(1) + i_ell(:, 1), 'b', 'Linewidth', 2);

CE_last_ell = create_probability_contour(P_CE_last);
plot(ax, mu_CE_last(2), mu_CE_last(1), 'cx', 'Linewidth', 2);
plot(ax, CE_last_ell(:, 2) + mu_CE_last(2), CE_last_ell(:, 1) + mu_CE_last(1), 'c', 'Linewidth', 1.6);

h_muCE = []; h_PCE = [];
h_s = []; h_es = [];

