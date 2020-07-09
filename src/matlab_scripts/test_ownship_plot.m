figure; axis equal
[~, n_samples] = size(X);
th = 0 : 0.01 : 2.01 * pi;
x_a = 20 * cos(th); y_a = 20 * sin(th);
x_cs = 50 * cos(th); y_cs = 50 * sin(th);
hold on; grid on;
plot(X(2, :), X(1, :), 'k');
for nwp = 1 : length(WPs(1, :))
    plot(y_a + WPs(2, nwp), x_a + WPs(1, nwp), 'c');
end

plot(WPs(2, :), WPs(1, :), 'rx', 'Linewidth', 1.2);

for k = 1 : 50 : n_samples
    plot(y_cs + X(2, k), x_cs + X(1, k), 'r');
end
legend('OS', 'WPs');