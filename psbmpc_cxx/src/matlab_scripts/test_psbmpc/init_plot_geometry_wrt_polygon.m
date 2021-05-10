
polygon = polyshape(polygon_vertices');
bbox = polyshape(poly_bbox');

fig = figure; axis equal
hold on; grid on;
ax1 = gca;
plot(ax1, polygon_vertices(2, :), polygon_vertices(1, :), 'k');
plot(ax1, poly_bbox(2, :), poly_bbox(1, :), 'g');
h_p_os_ray = plot(ax1, p_os_ray(2, 1), p_os_ray(1, 1), 'kx', 'linewidth', 3);
h_p_os = plot(ax1, p_os_ray(2, :), p_os_ray(1, :), 'b');

h_polygon_side = [];