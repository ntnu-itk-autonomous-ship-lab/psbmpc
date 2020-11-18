figure(1); axis equal
hold on; grid on;
[~, n_samples] = size(X);
th = 0 : 0.01 : 2.01 * pi;
x_a = 20 * cos(th); y_a = 20 * sin(th);
x_cs = 50 * cos(th); y_cs = 50 * sin(th);

plot(X(2, :), X(1, :), 'k', 'Linewidth', 1.2);
for nwp = 1 : length(WPs(1, :))
    plot(y_a + WPs(2, nwp), x_a + WPs(1, nwp), 'c');
end

plot(WPs(2, :), WPs(1, :), 'rx', 'Linewidth', 1.2);

t = dt_sim * (0 : n_samples - 1);
figure(2);
hold on; grid on;
plot(t, X(3, :) * 180 / pi, 'k', 'Linewidth', 1.2);
xlabel('Time [s]'); ylabel('COG [deg]');

figure(3);
hold on; grid on;
plot(t, X(4, :), 'k', 'Linewidth', 1.2);
xlabel('Time [s]'); ylabel('SOG [m/s]');

