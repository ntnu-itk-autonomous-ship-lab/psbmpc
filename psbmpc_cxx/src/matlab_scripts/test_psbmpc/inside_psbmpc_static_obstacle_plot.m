
h_text_j{j} = text(ax1, polygon_matrix_j(2, 1) + 10, polygon_matrix_j(1, 1) + 10, ['j=', num2str(j)]);
h_polygon_j{j} = plot(ax1, polygon_matrix_j(2, :), polygon_matrix_j(1, :));

p_os = [ownship_state(1); ownship_state(2)];
p_os_plus = [ownship_state(1) + d_0j(1); ownship_state(2) + d_0j(2)];
h_d_j{j} = plot(ax1, [p_os(2) p_os_plus(2)], [p_os(1) p_os_plus(1)]);