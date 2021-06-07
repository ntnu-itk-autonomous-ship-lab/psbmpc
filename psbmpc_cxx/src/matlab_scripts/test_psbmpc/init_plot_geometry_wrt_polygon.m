
poly_bbox = zeros(2, 5);
poly_bbox(:, 1) = bbox(:, 1);
poly_bbox(1, 2) = bbox(1, 1);
poly_bbox(2, 2) = bbox(2, 2);
poly_bbox(1, 3) = bbox(1, 2);
poly_bbox(2, 3) = bbox(2, 2);
poly_bbox(1, 4) = bbox(1, 2);
poly_bbox(2, 4) = bbox(2, 1);
poly_bbox(:, 5) = bbox(:, 1);

fig = figure; axis equal
hold on; grid on;
ax1 = gca;
plot(ax1, polygon_vertices(2, :), polygon_vertices(1, :), 'k');
plot(ax1, poly_bbox(2, :), poly_bbox(1, :), 'g');
h_p_os_ray = plot(ax1, p_os_ray(2, 1), p_os_ray(1, 1), 'kx', 'linewidth', 3);
h_p_os = plot(ax1, p_os_ray(2, :), p_os_ray(1, :), 'b');

h_polygon_side = [];