fig = figure;
ax1 = axes(fig, 'Position',[0.1 0.12 0.7 0.8]);
axis equal

ylimits_ne = [-100 1200];
xlimits_ne = [-600 600];
xlim(ax1, xlimits_ne);
ylim(ax1, ylimits_ne);

tlim = [0 T_sim];

warning('off', 'MATLAB:legend:IgnoringExtraEntries');

n_samples = T_sim / dt_sim;

t_vec = dt_sim * (0 : (n_samples - 1));

Lx = 0.02*ylimits_ne(2);
Ly = 0.002*(xlimits_ne(2) - xlimits_ne(1));
% drawn boat dimensions
boat_dim = [Lx/2 .7*Lx/2 -Lx/2 -Lx/2 .7*Lx/2 Lx/2; 0 2*Ly 2*Ly  -2*Ly -2*Ly 0];

colors = cell(1,20);
colors{1} = rgb('Green'); colors{2} = rgb('Orange'); colors{3} = rgb('Purple'); colors{4} = rgb('Yellow');
colors{5} = rgb('Brown'); colors{6} = rgb('DarkSlateGrey'); colors{7} = rgb('Gray'); colors{8} = rgb('MidnightBlue');
colors{9} = rgb('DarkRed'); colors{10} = rgb('DeepPink'); colors{11} = rgb('Chocolate'); colors{12} = rgb('Olive');
colors{13} = rgb('BlueViolet'); colors{14} = rgb('Salmon'); colors{15} = rgb('MistyRose'); colors{16} = rgb('DeepPink');

legend_strs = strings(1, n_ps + 1);
for ps = 1: n_ps
   legend_strs(ps) = ['ps=' num2str(ps - 1)]; 
end

% ax1: NE plot, ax2 & 3: mrou ne speeds vs mean speeds
ax2 = axes(fig, 'Position',[0.71 0.6 0.29 0.30]);
ax3 = axes(fig, 'Position',[0.71 0.2 0.29 0.30]);

xlim(ax2, tlim);
xlim(ax3, tlim);

hold(ax1, 'on'); hold(ax2, 'on'); hold(ax3, 'on');
grid(ax1, 'on'); grid(ax2, 'on'); grid(ax3, 'on'); 

ylabel(ax1,'North [m]');  xlabel(ax1,'East [m]'); 
ylabel(ax2,'Speed [m/s]');  xlabel(ax2,'Time [s]');
ylabel(ax3,'Speed [m/s]');  xlabel(ax3,'Time [s]');

h_X_i_ps = cell(n_ps, 1);
h_X_i_ps_ptch = cell(n_ps, 1);
h_P_i_ps = cell(n_ps, 1);
h_gt_Vx_i_ps = cell(n_ps, 1);
h_mean_Vx_i_ps = cell(n_ps, 1);
h_gt_Vy_i_ps = cell(n_ps, 1);
h_mean_Vy_i_ps = cell(n_ps, 1);

for ps = 1 : n_ps
    h_X_i_ps{ps} = [];
    h_X_i_ps_ptch{ps} = [];
    h_P_i_ps{ps} = [];
    h_gt_Vx_i_ps{ps} = [];
    h_mean_Vx_i_ps{ps} = [];
    h_gt_Vy_i_ps{ps} = [];
    h_mean_Vy_i_ps{ps} = [];
end
