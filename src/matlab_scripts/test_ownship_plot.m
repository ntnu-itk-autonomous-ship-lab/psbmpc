figure; axis equal
th = 0 : 0.01 : 2.01 * pi;
x_a = 20 * cos(th); y_a = 20 * sin(th);
hold on; grid on;
plot(X(2, :), X(1, :), 'k');
for nwp = 1 : length(WPs(1, :))
    plot(y_a + WPs(2, nwp), x_a + WPs(1, nwp), 'c');
end

plot(WPs(2, :), WPs(1, :), 'rx', 'Linewidth', 1.2);
legend('OS', 'WPs');