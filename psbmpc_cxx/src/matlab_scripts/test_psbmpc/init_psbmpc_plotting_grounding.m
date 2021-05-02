fig = figure; axis equal
hold on; grid on;
ax1 = axes(fig, 'Position',[0.1 0.1 0.6 0.8]);
origin = [270250, 7042250]; % nidelva north of ravnkloa
ylimits_ne = [origin(2) - 1000, origin(2) + 2000];
xlimits_ne = [origin(1) - 1500, origin(1) + 1500];
xlim(ax1, xlimits_ne);
ylim(ax1, ylimits_ne);


ylimits_d = [0,  1000];
tlim = [0 T_sim];

% drawn boat dimensions
boat_dim = [10 7 -10 -10 7 10; 0 2.4 2.4 -2.4 -2.4 0];

ax2 = axes(fig, 'Position',[0.75 0.55 0.25 0.35]);
ylim(ax2, ylimits_d); xlim(ax2, tlim);

hold(ax1, 'on'); hold(ax2, 'on');
grid(ax1, 'on'); grid(ax2, 'on');

P(P == -1) = NaN;
test_polygon=polyshape(P);

plot(ax1,test_polygon);

ylabel(ax1,'North [m]');  xlabel(ax1,'East [m]'); 
ylabel(ax2,'Distance [m]');  xlabel(ax2,'Time [s]');

th = 0 : 0.01 : 2.01 * pi;
x_cs = cos(th); y_cs = sin(th);

d_safe = 50;

h_wps = plot(ax1, WPs(2, :), WPs(1, :), 'rx', 'Linewidth', 2);

h_X = []; h_X_p = [];
h_X_text = []; h_X_k = [];
h_X_ptch = [];
h_safe = []; h_safe_k = [];

h_d_i = cell(n_obst, 1);
h_X_i = cell(n_obst, 1);
h_X_i_ptch = cell(n_obst, 1);
h_P_i = cell(n_obst, 1);
for i = 1 : n_obst
    h_d_i{i} = [];
    h_X_i{i} = [];
    h_X_i_ptch{i} = [];
    h_P_i{i} = [];
end

drawnow;