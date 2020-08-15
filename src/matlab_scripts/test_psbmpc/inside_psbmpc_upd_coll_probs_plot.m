delete(h_P_c_i{i, ps});

h_P_c_i{i, ps} = plot(ax2, t_vec, P_c_i(ps, :), 'Linewidth', 1.6, 'DisplayName', char(['(i,ps)=', num2str(i), ",", num2str(ps)]));

legend(ax2, 'show', 'location', 'northeast');
