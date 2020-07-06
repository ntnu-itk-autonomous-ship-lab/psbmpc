
ax = gca;
delete(h_muCE); delete(h_PCE); delete(h_s); delete(h_es);
h_s = plot(ax, samples(2, :), samples(1, :), 'kx');
h_es = plot(ax, elite_samples(2, :), elite_samples(1, :), 'gx');

h_muCE = plot(ax, mu_CE(2), mu_CE(1), 'mx', 'Linewidth', 2);

CE_ell = create_probability_contour(P_CE);
h_PCE = plot(ax, CE_ell(:, 2) + mu_CE(2), CE_ell(:, 1) + mu_CE(1), 'm', 'Linewidth', 1.6);
drawnow;
