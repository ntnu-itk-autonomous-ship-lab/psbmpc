figure; axis equal
hold on; grid on;
ylimits_ne = [0 1000];
xlimits_ne = [-300 300];
xlimits(gca, xlimits_ne);
ylimits(gca, ylimits_ne);

th = 0 : 0.01 : 2.01 * pi;
x_a = 20 * cos(th); y_a = 20 * sin(th);

for nwp = 1 : length(WPs(1, :))
    plot(y_a + WPs(2, nwp), x_a + WPs(1, nwp), 'c');
end

plot(WPs(2, :), WPs(1, :), 'rx', 'Linewidth', 1.2);

h_X = [];
h_X_ptch = [];
h_safe = [];
