fig = figure; axis equal
hold on; grid on;
ax1 = gca; 
set(ax1, 'Position',[0.1 0.1 0.6 0.8]);
origin = [270250, 7042250]; % nidelva north of ravnkloa
ylimits_ne = [origin(2) - 1000, origin(2) + 2000];
xlimits_ne = [origin(1) - 1500, origin(1) + 1500];
xlim(ax1, xlimits_ne);
ylim(ax1, ylimits_ne);


ylimits_d = [0,  1000];
tlim = [0 T_sim / dt_sim];

% L_os = 0.04*ylimits(2);
% w_os = 0.006*(xlimits(2) - xlimits(1));
% 
% L_i = 0.04*ylimits(2);
% w_i = 0.006*(xlimits(2) - xlimits(1));
% % drawn boat dimensions
L_os = 5; L_i = 5;
w_os = 3; w_i = 3;
boat_dim = [L_os/2 .7*L_os/2 -L_os/2 -L_os/2 .7*L_os/2 L_os/2; 0 2*w_os 2*w_os  -2*w_os -2*w_os 0];


ax2 = axes(fig, 'Position',[0.7 0.2 0.22 0.5]);
ylim(ax2, ylimits_d); xlim(ax2, tlim);

hold(ax1, 'on'); hold(ax2, 'on');
grid(ax1, 'on'); grid(ax2, 'on');

n_samples = T_sim / dt_sim;
t_vec = dt_sim * (0 : (n_samples - 1));
plot(ax2, t_vec, d_safe * ones(n_samples, 1), 'r');

disp(map_origin);
[n_vertices, ~] = size(P)
P(P == -1) = NaN;
P(:, 1) = P(:, 1) + map_origin(2);
P(:, 2) = P(:, 2) + map_origin(1);
polygon=polyshape(P);

[n_vertices_simplified, ~] = size(P_simplified)
P_simplified(P_simplified == -1) = NaN;
P_simplified(:, 1) = P_simplified(:, 1) + map_origin(2);
P_simplified(:, 2) = P_simplified(:, 2) + map_origin(1);
simplified_polygon = polyshape(P_simplified);

plot(ax1, polygon); plot(ax1, simplified_polygon);

ylabel(ax1,'North [m]');  xlabel(ax1,'East [m]'); 
ylabel(ax2,'Distance [m]');  xlabel(ax2,'Time [s]');

th = 0 : 0.01 : 2.01 * pi;
x_cs = cos(th); y_cs = sin(th);

h_wps = plot(ax1, map_origin(2) + WPs(2, :), map_origin(1) + WPs(1, :), 'kx', 'Linewidth', 2);

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


