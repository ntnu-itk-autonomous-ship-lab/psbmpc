fig = figure(2); clf; axis equal
hold on; grid on;
ax1 = gca;
ylimits_ne = [-100 1000];
xlimits_ne = [-600 600];
xlim(ax1, xlimits_ne);
ylim(ax1, ylimits_ne);

colors = cell(1,20);
colors{1} = rgb('Green'); colors{2} = rgb('Orange'); colors{3} = rgb('Purple'); colors{4} = rgb('Yellow');
colors{5} = rgb('Brown'); colors{6} = rgb('DarkSlateGrey'); colors{7} = rgb('Gray'); colors{8} = rgb('MidnightBlue');
colors{9} = rgb('DarkRed'); colors{10} = rgb('DeepPink'); colors{11} = rgb('Chocolate'); colors{12} = rgb('Olive');
colors{13} = rgb('BlueViolet'); colors{14} = rgb('Salmon'); colors{15} = rgb('MistyRose'); colors{16} = rgb('DeepPink');

legend_strs = strings(1, n_obst + 1);
legend_strs(1) = 'Safe dist.';
for i = 2:n_obst + 1
   legend_strs(i) = ['i = ' num2str(i-1)]; 
end

%ylimits_d = [0 500];
n_samples = round(T_sim / dt_sim);
tlim = [0 T_sim];
t_vec = 0 : dt_sim : (n_samples - 1) * dt_sim;

Lx = 0.02*ylimits_ne(2);
Ly = 0.002*(xlimits_ne(2) - xlimits_ne(1));
% drawn boat dimensions
boat_dim = [Lx/2 .7*Lx/2 -Lx/2 -Lx/2 .7*Lx/2 Lx/2; 0 2*Ly 2*Ly  -2*Ly -2*Ly 0];

hold(ax1, 'on');
grid(ax1, 'on'); 

ylabel(ax1,'North [m]');  xlabel(ax1,'East [m]'); 

th = 0 : 0.01 : 2.01 * pi;
x_cs = cos(th); y_cs = sin(th);

d_ij = cell(n_obst, n_obst); 

h_X = [];
h_X_ptch = [];
h_safe = []; 

obs = cell(n_obst, 1);
for i = 1 : n_obst
    obs{i}.X = zeros(4, n_samples);
    obs{i}.X_pred = cell(1, n_samples);
end

h_wps_i = cell(n_obst, 1);
h_X_i = cell(n_obst, 1);
h_X_i_p = cell(n_obst, 1);
h_X_i_ptch = cell(n_obst, 1);
h_text_i = cell(n_obst, 1);
h_safe_i = cell(n_obst, 1);
for i = 1 : n_obst
    h_wps_i{i} = [];
    h_X_i{i} = []; 
    h_X_i_p{i} = [];
    h_X_i_ptch{i} = [];
    h_text_i{i} = [];
    h_safe_i{i} = [];
end

h_X_static = cell(n_static_obst, 1);
% so_x = zeros(2, 1); so_y = so_x;
% for j = 1 : n_static_obst
%     so_x(1) = X_static(1, j); so_x(2) = X_static(3, j); 
%     so_y(1) = X_static(2, j); so_y(2) = X_static(4, j); 
%     h_X_static{i} = plot(ax1, so_y, so_x, 'm', 'Linewidth', 3); 
% end

drawnow;


