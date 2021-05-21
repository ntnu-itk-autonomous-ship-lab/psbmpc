fig = figure; axis equal
hold on; grid on;
ax1 = gca; 
set(ax1, 'Position',[0.1 0.1 0.6 0.8]);
origin = [270250, 7042250]; % nidelva north of ravnkloa
ylimits_ne = [origin(2) - 7000, origin(2) + 7000];
xlimits_ne = [origin(1) - 7000, origin(1) + 7000];
xlim(ax1, xlimits_ne);
ylim(ax1, ylimits_ne);

% drawn boat dimensions
boat_dim = [10 7 -10 -10 7 10; 0 2.4 2.4 -2.4 -2.4 0];

hold(ax1, 'on');
grid(ax1, 'on');

disp(map_origin);
[n_vertices, ~] = size(P)
P(P == -1e6) = NaN;
save('Pmat1', 'P');
P(:, 1) = P(:, 1) + map_origin(2);
P(:, 2) = P(:, 2) + map_origin(1);
polygon = polyshape(P);
plot(ax1, polygon); 

[n_vertices_simplified, ~] = size(P_simplified)
P_simplified(P_simplified == -1e6) = NaN;
save('Pmat2', 'P_simplified');
P_simplified(:, 1) = P_simplified(:, 1) + map_origin(2);
P_simplified(:, 2) = P_simplified(:, 2) + map_origin(1);
simplified_polygon = polyshape(P_simplified);
plot(ax1, simplified_polygon);

plot(ax1, map_origin(2), map_origin(1), 'rx', 'linewidth', 3);

ylabel(ax1,'North [m]');  xlabel(ax1,'East [m]'); 
