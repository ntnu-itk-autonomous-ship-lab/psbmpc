fig = figure(2); clf; axis equal
hold on; grid on;
ax1 = gca;
ylimits_ne = [-400 1000];
xlimits_ne = [-600 600];
xlim(ax1, xlimits_ne);
ylim(ax1, ylimits_ne);

%ylimits_d = [0 500];
%tlim = [0 T_sim];

Lx = 0.02*ylimits_ne(2);
Ly = 0.002*(xlimits_ne(2) - xlimits_ne(1));
% drawn boat dimensions
boat_dim = [Lx/2 .7*Lx/2 -Lx/2 -Lx/2 .7*Lx/2 Lx/2; 0 2*Ly 2*Ly  -2*Ly -2*Ly 0];

hold(ax1, 'on'); 
grid(ax1, 'on'); 

ylabel(ax1,'North [m]');  xlabel(ax1,'East [m]'); 

th = 0 : 0.01 : 2.01 * pi;
x_cs = cos(th); y_cs = sin(th);

d_safe = 50;

h_wps = plot(ax1, WPs(2, :), WPs(1, :), 'rx', 'Linewidth', 2);

h_X = []; h_X_p = [];
h_X_text = []; h_X_k = [];
h_X_ptch = [];
h_safe = []; h_safe_k = [];

h_X_i = cell(n_obst, n_ps);
h_X_i_s = cell(n_obst, n_ps);
h_X_i_ptch = cell(n_obst, n_ps);
h_P_i = cell(n_obst, n_ps);
h_text_i = cell(n_obst, n_ps);
for i = 1 : n_obst
    for ps = 1 : n_ps
        h_X_i{i, ps} = [];
        h_X_i_s{i, ps} = [];
        h_X_i_ptch{i, ps} = [];
        h_P_i{i, ps} = [];
        h_text_i{i, ps} = [];
    end
end

