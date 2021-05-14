fig = figure(2); clf; axis equal
hold on; grid on;
ax1 = gca;
ylimits_ne = [-1000 2000];
xlimits_ne = [-1500 1500];
xlim(ax1, xlimits_ne);
ylim(ax1, ylimits_ne);

ax2 = axes(fig, 'Position',[0.72 0.15 0.28 0.35]);


%ylimits_d = [0 500];
n_samples = round(T_sim / dt_sim);
tlim = [0 T_sim];
t_vec = 0 : dt_sim : (n_samples - 1) * dt_sim;
ylim(ax2, [-0.05 1.05]); %xlim(ax2, tlim);

Lx = 0.01*ylimits_ne(2);
Ly = 0.001*(xlimits_ne(2) - xlimits_ne(1));
% drawn boat dimensions
boat_dim = [Lx/2 .7*Lx/2 -Lx/2 -Lx/2 .7*Lx/2 Lx/2; 0 2*Ly 2*Ly  -2*Ly -2*Ly 0];

hold(ax1, 'on'); hold(ax2, 'on'); 
grid(ax1, 'on'); grid(ax2, 'on'); 

ylabel(ax1,'North [m]');  xlabel(ax1,'East [m]'); 
ylabel(ax2,'Probability');  xlabel(ax2,'Time [s]'); 

[n_vertices, ~] = size(P);
P(P == -1) = NaN;
test_polygon=polyshape(P);

plot(ax1,test_polygon);

th = 0 : 0.01 : 2.01 * pi;
x_cs = cos(th); y_cs = sin(th);

d_safe = 50;

h_wps = plot(ax1, WPs(2, :), WPs(1, :), 'rx', 'Linewidth', 2);

h_X = []; h_X_p = [];
h_X_ptch = [];
h_safe = []; 

h_X_text_s = cell(n_samples, 1); h_X_s = cell(n_samples, 1);
h_safe_s = cell(n_samples,1);
for s = 1 : n_samples
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

drawnow;


