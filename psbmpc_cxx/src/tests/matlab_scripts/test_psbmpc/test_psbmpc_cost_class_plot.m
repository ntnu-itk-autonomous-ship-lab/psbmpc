fig = figure; axis equal
hold on; grid on;
ax1 = gca;
origin = [270250, 7042250]; % nidelva north of ravnkloa
ylimits_ne = [origin(2) - 10000, origin(2) + 10000];
xlimits_ne = [origin(1) - 10000, origin(1) + 10000];
xlim(ax1, xlimits_ne);
ylim(ax1, ylimits_ne);


ylimits_d = [0,  1000];
tlim = [0 T_sim];

% drawn boat dimensions
boat_dim = [10 7 -10 -10 7 10; 0 2.4 2.4 -2.4 -2.4 0];

hold(ax1, 'on');
grid(ax1, 'on');
ylabel(ax1,'North [m]');  xlabel(ax1,'East [m]'); 

disp(map_origin);
[n_vertices, ~] = size(P)
P(P == -1) = NaN;
P(:, 1) = P(:, 1) + map_origin(2);
P(:, 2) = P(:, 2) + map_origin(1);

test_polygon=polyshape(P)

plot(ax1, test_polygon);
plot(ax1, selected_polygon_matrix(:, 1), selected_polygon_matrix(:, 2), 'm');

[ll, n_static_obst] = size(d2poly_cpu);

plot(ax1, map_origin(2) + X(2, 1), map_origin(1) + X(1, 1), 'gx', 'linewidth', 5);

quiver(ax1, map_origin(2) + X(2, 1) * ones(1, n_static_obst), map_origin(1) + X(1, 1) * ones(1, n_static_obst), d2poly_gpu(2, :), d2poly_gpu(1, :));

th = 0 : 0.01 : 2.01 * pi;
x_cs = cos(th); y_cs = sin(th);

d_safe = 50;

h_wps = plot(ax1, map_origin(2) +  WPs(2, :), map_origin(1) + WPs(1, :), 'rx', 'Linewidth', 2);
