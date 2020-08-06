fig = figure(2); clf; axis equal
hold on; grid on;
ax1 = gca;
ylimits_ne = [-400 1000];
xlimits_ne = [-600 600];
xlim(ax1, xlimits_ne);
ylim(ax1, ylimits_ne);

ax2 = axes(fig, 'Position',[0.72 0.55 0.26 0.35]);


%ylimits_d = [0 500];
tlim = [0 T_sim];
ylim(ax2, [-0.05 1.05]); %xlim(ax2, tlim);

Lx = 0.02*ylimits_ne(2);
Ly = 0.002*(xlimits_ne(2) - xlimits_ne(1));
% drawn boat dimensions
boat_dim = [Lx/2 .7*Lx/2 -Lx/2 -Lx/2 .7*Lx/2 Lx/2; 0 2*Ly 2*Ly  -2*Ly -2*Ly 0];

hold(ax1, 'on'); hold(ax2, 'on'); 
grid(ax1, 'on'); grid(ax2, 'on'); 

ylabel(ax1,'North [m]');  xlabel(ax1,'East [m]'); 
ylabel(ax2,'Probability');  xlabel(ax1,'Sample'); 

th = 0 : 0.01 : 2.01 * pi;
x_cs = cos(th); y_cs = sin(th);

d_safe = 50;

h_wps = plot(ax1, WPs(2, :), WPs(1, :), 'rx', 'Linewidth', 2);

h_X = []; h_X_p = [];
h_X_ptch = [];
h_safe = []; 

% 100 is a safe max number of plot points.
h_X_text_s = cell(100, 1); h_X_s = cell(100, 1);
h_safe_s = cell(100,1);
for s = 1 : 1000
    h_X_text_s{s, 1} = [];
    h_safe_s{s, 1} = [];
    h_X_s{s, 1} = [];
end

h_X_i_s = cell(n_obst, n_ps);
h_X_i = cell(n_obst, n_ps);
h_X_i_ptch = cell(n_obst, n_ps);

h_P_i = cell(n_obst, n_ps);
h_P_c_i = cell(n_obst, n_ps);
h_text_i = cell(n_obst, n_ps);
for i = 1 : n_obst
    for ps = 1 : n_ps
        h_X_i{i, ps} = [];
        h_X_i_s{i, ps} = [];
        h_X_i_ptch{i, ps} = [];
        h_P_i{i, ps} = [];
        h_P_c_i{i, ps} = [];
        h_text_i{i, ps} = [];
    end
end

